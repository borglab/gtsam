/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file PriorityScheduler.h
 * @brief Priority-based task scheduler.
 *
 * @details
 * This header defines a small, thread-based scheduler that executes tasks in
 * priority order. A lower numeric priority is executed before a higher numeric
 * priority (min-heap behavior). Tasks return a value of type `Y`, and callers
 * can wait on results via `std::future<Y>`.
 *
 * This is implemented as a policy specialization of `gtsam::Scheduler`.
 *
 * @par Example
 * @code
 * gtsam::PriorityScheduler<int> scheduler(4);
 * auto future = scheduler.schedule(0, [] { return 42; });
 * int value = future.get();
 * @endcode
 *
 * @author Frank Dellaert
 * @date May, 2025
 */

#pragma once

#include <gtsam/base/Scheduler.h>

namespace gtsam {

namespace detail {

/// Queue policy for PriorityScheduler: min-heap priority_queue by task
/// priority.
struct PrioritySchedulerPolicy {
  using Metadata = int;

  template <typename TaskPtr>
  struct Compare {
    bool operator()(const TaskPtr& a, const TaskPtr& b) const {
      return a.metadata > b.metadata;
    }
  };

  template <typename TaskPtr>
  using Container =
      std::priority_queue<TaskPtr, std::vector<TaskPtr>, Compare<TaskPtr>>;

  template <typename TaskPtr>
  static void push(Container<TaskPtr>& container, TaskPtr task) {
    container.push(std::move(task));
  }

  template <typename TaskPtr>
  static bool popLocal(Container<TaskPtr>& container, TaskPtr& out) {
    if (container.empty()) return false;
    out = container.top();
    container.pop();
    return true;
  }

  template <typename TaskPtr>
  static bool popSteal(Container<TaskPtr>& container, TaskPtr& out) {
    return popLocal(container, out);
  }
};

}  // namespace detail

/**
 * @brief Thread pool scheduler that prioritizes tasks by numeric priority.
 *
 * @details
 * - Lower numeric values are executed first.
 * - Tasks are executed by worker threads created at construction.
 * - `schedule` returns a `std::future<Y>` for the task result.
 * - Per-thread queues reduce contention; workers steal from peers when idle.
 * - External submissions are round-robin distributed across worker queues.
 * - A condition variable parks workers when no work is available.
 *
 * @tparam Y Result type returned by tasks. Use `void` for no return value.
 */
template <typename Y>
using PriorityScheduler = Scheduler<Y, detail::PrioritySchedulerPolicy>;

}  // namespace gtsam
