/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TaskScheduler.h
 * @brief Cooperative task scheduler.
 *
 * @details
 * This header defines a small, thread-based scheduler that executes tasks in
 * a cooperative manner without priority ordering. Tasks return a value of type
 * `Y`, and callers can wait on results via `std::future<Y>`.
 *
 * This is implemented as a policy specialization of `gtsam::Scheduler`.
 *
 * @par Example
 * @code
 * gtsam::TaskScheduler<int> scheduler(4);
 * auto future = scheduler.schedule([] { return 42; });
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
/// Queue policy for TaskScheduler: deque with local LIFO and stolen FIFO.
struct TaskSchedulerPolicy {
  using Metadata = std::monostate;

  template <typename TaskPtr>
  using Container = std::deque<TaskPtr>;

  template <typename TaskPtr>
  static void push(Container<TaskPtr>& container, TaskPtr task) {
    container.push_back(std::move(task));
  }

  template <typename TaskPtr>
  static bool popLocal(Container<TaskPtr>& container, TaskPtr& out) {
    if (container.empty()) return false;
    out = std::move(container.back());
    container.pop_back();
    return true;
  }

  template <typename TaskPtr>
  static bool popSteal(Container<TaskPtr>& container, TaskPtr& out) {
    if (container.empty()) return false;
    out = std::move(container.front());
    container.pop_front();
    return true;
  }
};
}  // namespace detail

/**
 * @brief Thread pool scheduler that executes tasks without priority ordering.
 *
 * @details
 * - Tasks are executed in an efficient deque-based order (LIFO locally).
 * - Per-thread queues reduce contention; workers steal from peers when idle.
 * - External submissions are round-robin distributed across worker queues.
 * - A condition variable parks workers when no work is available.
 *
 * @tparam Y Result type returned by tasks. Use `void` for no return value.
 */
template <typename Y>
using TaskScheduler = Scheduler<Y, detail::TaskSchedulerPolicy>;

}  // namespace gtsam
