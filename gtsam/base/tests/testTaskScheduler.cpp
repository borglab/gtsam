/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testTaskScheduler.cpp
 * @brief Unit tests for TaskScheduler scheduling behavior.
 * @author Frank Dellaert
 * @date Jan, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TaskScheduler.h>

#include <atomic>
#include <mutex>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
// Compose child return values inside a parent task
TEST(TaskScheduler, ReturnValueComposition) {
  // Use multiple workers so parent tasks can wait without stalling the pool.
  TaskScheduler<size_t> scheduler(2);

  auto left = scheduler.schedule([] { return size_t{3}; }).share();
  auto right = scheduler.schedule([] { return size_t{4}; }).share();

  auto parent = scheduler.schedule(
      [left, right]() mutable { return left.get() + right.get(); });

  const size_t expectedSum = 7;
  EXPECT_LONGS_EQUAL(expectedSum, parent.get());
}

/* ************************************************************************* */
TEST(TaskScheduler, EnqueueFireAndForget) {
  TaskScheduler<void> scheduler(2);

  std::atomic<size_t> counter{0};
  const size_t taskCount = 256;

  for (size_t i = 0; i < taskCount; ++i) {
    scheduler.enqueue([&counter] { counter.fetch_add(1); });
  }

  scheduler.waitForAllTasks();
  EXPECT_LONGS_EQUAL(taskCount, counter.load());
}

/* ************************************************************************* */
TEST(TaskScheduler, EnqueueOrRunInlineOnWorker) {
  TaskScheduler<void> scheduler(1);

  std::atomic<size_t> inlineCounter{0};
  auto future = scheduler.schedule([&] {
    scheduler.enqueueOrRunInline([&] { inlineCounter.fetch_add(1); });
  });

  future.get();
  scheduler.waitForAllTasks();
  EXPECT_LONGS_EQUAL(1, inlineCounter.load());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
