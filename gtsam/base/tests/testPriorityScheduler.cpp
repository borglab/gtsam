/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testPriorityScheduler.cpp
 * @brief Unit tests for PriorityScheduler scheduling behavior.
 * @author Frank Dellaert
 * @date May, 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/PriorityScheduler.h>

#include <mutex>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
// Compose child return values inside a parent task
TEST(PriorityScheduler, ReturnValueComposition) {
  // Use multiple workers so parent tasks can wait without stalling the pool.
  PriorityScheduler<size_t> scheduler(2);

  // Schedule leaves first with higher priority (lower numeric value).
  auto left = scheduler.schedule(0, [] { return size_t{3}; }).share();
  auto right = scheduler.schedule(0, [] { return size_t{4}; }).share();

  // Parent task waits on the child futures and combines their values.
  auto parent = scheduler.schedule(
      10, [left, right]() mutable { return left.get() + right.get(); });

  const size_t expectedSum = 7;
  EXPECT_LONGS_EQUAL(expectedSum, parent.get());
}

/* ************************************************************************* */
TEST(PriorityScheduler, VoidPriorityOrderingSingleWorker) {
  PriorityScheduler<void> scheduler(1);

  std::mutex mutex;
  std::vector<int> executionOrder;

  scheduler.schedule(5, [&] {
    std::lock_guard<std::mutex> lock(mutex);
    executionOrder.push_back(5);
  });
  scheduler.schedule(1, [&] {
    std::lock_guard<std::mutex> lock(mutex);
    executionOrder.push_back(1);
  });
  scheduler.schedule(3, [&] {
    std::lock_guard<std::mutex> lock(mutex);
    executionOrder.push_back(3);
  });

  scheduler.waitForAllTasks();
  EXPECT_LONGS_EQUAL(3, executionOrder.size());
  // With a single worker, tasks are not preempted: the first scheduled task may
  // start running before later (higher-priority) tasks are submitted. Also,
  // since the third task is submitted last, the worker may run the first two
  // tasks before the third is enqueued.
  const bool strictPriority =
      (executionOrder.at(0) == 1 && executionOrder.at(1) == 3 &&
       executionOrder.at(2) == 5);
  const bool firstTaskRanImmediately =
      (executionOrder.at(0) == 5 && executionOrder.at(1) == 1 &&
       executionOrder.at(2) == 3);
  const bool thirdTaskEnqueuedLate =
      (executionOrder.at(0) == 1 && executionOrder.at(1) == 5 &&
       executionOrder.at(2) == 3);
  EXPECT(strictPriority || firstTaskRanImmediately || thirdTaskEnqueuedLate);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
