/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Scheduler.h
 * @brief Policy-based work-stealing scheduler.
 *
 * @details
 * This header defines a small, thread-based scheduler core that executes tasks
 * using per-worker queues with opportunistic work-stealing. The queue behavior
 * (e.g., LIFO/FIFO, priority ordering) is defined by a Policy type.
 *
 * Public convenience wrappers are provided in `TaskScheduler.h` and
 * `PriorityScheduler.h`.
 *
 * @author Frank Dellaert
 * @date May, 2025
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace gtsam {

/**
 * @brief Thread pool scheduler parameterized by a queue Policy.
 *
 * @details
 * - Tasks are executed by worker threads created at construction.
 * - Per-worker queues reduce contention; workers steal from peers when idle.
 * - External submissions are round-robin distributed across worker queues.
 * - A condition variable parks workers when no work is available.
 *
 * Policy requirements:
 * - `using Metadata = ...;` carried with each task (e.g., `std::monostate`,
 * `int`)
 * - `template <typename TaskPtr> using Container = ...;`
 * - `push(container, task)`, `popLocal(container, out)`, `popSteal(container,
 * out)`
 *
 * @tparam Y Result type returned by tasks. Use `void` for no return value.
 * @tparam Policy Queue behavior policy.
 */
template <typename Y, typename Policy>
class Scheduler {
  using Metadata = typename Policy::Metadata;

  struct WorkItem {
    Metadata metadata{};
    std::function<void()> run;
  };

  using Container = typename Policy::template Container<WorkItem>;

  struct WorkerQueue {
    std::mutex mutex;
    Container queue;
  };

  std::vector<std::unique_ptr<WorkerQueue>> queues_;  // Per-worker queues.
  std::atomic<size_t> queuedTasks_{0};  // Tracks queued work for wakeups.
  std::vector<std::thread> workers_;
  mutable std::mutex waitMutex_;  // Dedicated mutex for condition_variable.
  std::condition_variable condition_;
  std::atomic<bool> stop_{false};
  std::atomic<int> activeTasks_{0};  // In-flight tasks (queued + running).
  inline static thread_local Scheduler* currentScheduler_ = nullptr;
  inline static thread_local int workerIndex_ = -1;
  std::atomic<size_t> nextWorker_{0};  // Round-robin distributor.

  /**
   * @brief Worker loop: wait for tasks, run them, and fulfill promises.
   *
   * Uses a condition variable to avoid spinning while the queue is empty.
   * Stops once `stop_` is set and no queued work remains.
   */
  void worker_thread(size_t index) {
    currentScheduler_ = this;
    workerIndex_ = static_cast<int>(index);
    while (true) {
      WorkItem item;
      if (!tryPopTask(index, item)) {
        std::unique_lock<std::mutex> lock(waitMutex_);
        condition_.wait(lock, [this] {
          return stop_.load(std::memory_order_acquire) ||
                 queuedTasks_.load(std::memory_order_acquire) > 0;
        });
        if (stop_.load(std::memory_order_acquire) &&
            queuedTasks_.load(std::memory_order_acquire) == 0) {
          currentScheduler_ = nullptr;
          workerIndex_ = -1;
          return;
        }
        continue;
      }

      try {
        item.run();
      } catch (...) {
        /* ignore */
      }

      // Notify waiters only when work transitions to "done".
      if (activeTasks_.fetch_sub(1, std::memory_order_release) == 1) {
        condition_.notify_all();
      }
    }
  }

  /// Attempt to get a task from local or stolen queues.
  bool tryPopTask(size_t index, WorkItem& item) {
    // Prefer local queue, then steal to keep workers busy.
    if (tryPopLocal(index, item)) return true;
    if (trySteal(index, item)) return true;
    return false;
  }

  /// Try to pop a task from the current worker queue.
  bool tryPopLocal(size_t index, WorkItem& item) {
    WorkerQueue& queue = *queues_[index];
    std::lock_guard<std::mutex> lock(queue.mutex);
    if (!Policy::template popLocal<WorkItem>(queue.queue, item)) return false;
    queuedTasks_.fetch_sub(1, std::memory_order_release);
    return true;
  }

  /// Try to steal a task from another worker queue.
  bool trySteal(size_t index, WorkItem& item) {
    const size_t workerCount = queues_.size();
    if (workerCount <= 1) return false;
    // Steal from other workers if local queue is empty.
    for (size_t offset = 1; offset < workerCount; ++offset) {
      size_t target = (index + offset) % workerCount;
      WorkerQueue& queue = *queues_[target];
      std::unique_lock<std::mutex> lock(queue.mutex, std::try_to_lock);
      if (!lock.owns_lock()) continue;
      if (!Policy::template popSteal<WorkItem>(queue.queue, item)) continue;
      queuedTasks_.fetch_sub(1, std::memory_order_release);
      return true;
    }
    return false;
  }

  /// Return true if called on a scheduler worker thread.
  bool isWorkerThread() const { return currentScheduler_ == this; }

  bool enqueueImpl(Metadata metadata, std::function<void()> work) {
    if (stop_.load(std::memory_order_acquire)) return false;
    WorkerQueue* targetQueue = nullptr;
    if (isWorkerThread()) {
      targetQueue = queues_[static_cast<size_t>(workerIndex_)].get();
    } else {
      const size_t target =
          nextWorker_.fetch_add(1, std::memory_order_relaxed) % queues_.size();
      targetQueue = queues_[target].get();
    }

    {
      std::lock_guard<std::mutex> lock(targetQueue->mutex);
      Policy::template push<WorkItem>(
          targetQueue->queue, WorkItem{std::move(metadata), std::move(work)});
    }

    queuedTasks_.fetch_add(1, std::memory_order_release);
    activeTasks_.fetch_add(1, std::memory_order_release);
    condition_.notify_one();
    return true;
  }

  template <typename T = Y, typename = std::enable_if_t<std::is_void_v<T>>>
  void scheduleOrRunInlineImpl(Metadata metadata, std::function<void()> job) {
    if (stop_.load(std::memory_order_relaxed)) return;
    if (!isWorkerThread()) {
      enqueueImpl(std::move(metadata), std::move(job));
      return;
    }

    activeTasks_.fetch_add(1, std::memory_order_release);
    try {
      job();
    } catch (...) { /* ignore */
    }
    if (activeTasks_.fetch_sub(1, std::memory_order_release) == 1) {
      condition_.notify_all();
    }
  }

 public:
  /**
   * @brief Construct a scheduler with a fixed number of worker threads.
   *
   * @param numThreads Number of worker threads to create. If zero, a single
   * thread is created.
   */
  explicit Scheduler(size_t numThreads = std::thread::hardware_concurrency()) {
    if (numThreads == 0) numThreads = 1;
    queues_.reserve(numThreads);
    for (size_t i = 0; i < numThreads; ++i) {
      queues_.push_back(std::make_unique<WorkerQueue>());
    }
    for (size_t i = 0; i < numThreads; ++i) {
      workers_.emplace_back(&Scheduler::worker_thread, this, i);
    }
  }

  /**
   * @brief Wait for all tasks to finish, then stop worker threads.
   *
   * @note The destructor calls `waitForAllTasks` before stopping workers.
   */
  ~Scheduler() {
    waitForAllTasks();
    stop_.store(true, std::memory_order_release);
    condition_.notify_all();
    for (std::thread& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  /**
   * @brief Enqueue a task for execution (no metadata).
   *
   * Only enabled when `Policy::Metadata` is `std::monostate`.
   *
   * @param job Callable returning a `Y`.
   * @return `std::future<Y>` associated with the task.
   */
  template <typename P = Policy, typename = std::enable_if_t<std::is_same_v<
                                     typename P::Metadata, std::monostate>>>
  std::future<Y> schedule(std::function<Y()> job) {
    if (stop_.load(std::memory_order_acquire)) {
      std::promise<Y> err_promise;
      err_promise.set_exception(std::make_exception_ptr(
          std::runtime_error("Scheduler is stopping or stopped.")));
      return err_promise.get_future();
    }

    auto promise = std::make_shared<std::promise<Y>>();
    std::future<Y> future = promise->get_future();

    auto work = [promise, job = std::move(job)]() mutable {
      try {
        if constexpr (std::is_void_v<Y>) {
          job();
          promise->set_value();
        } else {
          promise->set_value(job());
        }
      } catch (...) {
        try {
          promise->set_exception(std::current_exception());
        } catch (...) { /* ignore */
        }
      }
    };

    if (!enqueueImpl(Metadata{}, std::move(work))) {
      try {
        promise->set_exception(std::make_exception_ptr(
            std::runtime_error("Scheduler is stopping or stopped.")));
      } catch (...) { /* ignore */
      }
    }
    return future;
  }

  /**
   * @brief Enqueue a task for execution with metadata.
   *
   * Enabled when `Policy::Metadata` is not `std::monostate`.
   *
   * @param metadata Policy-defined metadata for ordering (e.g., priority).
   * @param job Callable returning a `Y`.
   * @return `std::future<Y>` associated with the task.
   */
  template <typename P = Policy, typename = std::enable_if_t<!std::is_same_v<
                                     typename P::Metadata, std::monostate>>>
  std::future<Y> schedule(Metadata metadata, std::function<Y()> job) {
    if (stop_.load(std::memory_order_acquire)) {
      std::promise<Y> err_promise;
      err_promise.set_exception(std::make_exception_ptr(
          std::runtime_error("Scheduler is stopping or stopped.")));
      return err_promise.get_future();
    }

    auto promise = std::make_shared<std::promise<Y>>();
    std::future<Y> future = promise->get_future();

    auto work = [promise, job = std::move(job)]() mutable {
      try {
        if constexpr (std::is_void_v<Y>) {
          job();
          promise->set_value();
        } else {
          promise->set_value(job());
        }
      } catch (...) {
        try {
          promise->set_exception(std::current_exception());
        } catch (...) { /* ignore */
        }
      }
    };

    if (!enqueueImpl(std::move(metadata), std::move(work))) {
      try {
        promise->set_exception(std::make_exception_ptr(
            std::runtime_error("Scheduler is stopping or stopped.")));
      } catch (...) { /* ignore */
      }
    }
    return future;
  }

  /**
   * @brief Schedule or run inline when called from a worker thread.
   *
   * Used to fuse continuations without re-entering the queues.
   */
  template <typename T = Y,
            typename = std::enable_if_t<
                std::is_void_v<T> && std::is_same_v<Metadata, std::monostate>>>
  void scheduleOrRunInline(std::function<void()> job) {
    scheduleOrRunInlineImpl(Metadata{}, std::move(job));
  }

  /**
   * @brief Schedule or run inline when called from a worker thread.
   *
   * Used to fuse continuations without re-entering the queues.
   */
  template <typename T = Y,
            typename = std::enable_if_t<
                std::is_void_v<T> && !std::is_same_v<Metadata, std::monostate>>>
  void scheduleOrRunInline(Metadata metadata, std::function<void()> job) {
    scheduleOrRunInlineImpl(std::move(metadata), std::move(job));
  }

  /**
   * @brief Enqueue a fire-and-forget task for execution.
   *
   * Enabled only for `TaskScheduler<void>` (i.e., `Y = void` and no metadata).
   */
  template <typename T = Y,
            typename = std::enable_if_t<
                std::is_void_v<T> && std::is_same_v<Metadata, std::monostate>>>
  void enqueue(std::function<void()> job) {
    enqueueImpl(Metadata{}, std::move(job));
  }

  /**
   * @brief Enqueue a fire-and-forget task or run it inline on worker threads.
   *
   * Enabled only for `TaskScheduler<void>` (i.e., `Y = void` and no metadata).
   */
  template <typename T = Y,
            typename = std::enable_if_t<
                std::is_void_v<T> && std::is_same_v<Metadata, std::monostate>>>
  void enqueueOrRunInline(std::function<void()> job) {
    scheduleOrRunInlineImpl(Metadata{}, std::move(job));
  }

  /**
   * @brief Block until all queued and active tasks complete.
   *
   * @note If the scheduler is stopping, this returns early.
   */
  void waitForAllTasks() {
    std::unique_lock<std::mutex> lock(waitMutex_);
    condition_.wait(lock, [this] {
      return stop_.load(std::memory_order_acquire) ||
             (activeTasks_.load(std::memory_order_acquire) == 0 &&
              queuedTasks_.load(std::memory_order_acquire) == 0);
    });
  }
};

}  // namespace gtsam
