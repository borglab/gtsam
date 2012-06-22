/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholesky.cpp
 * @author  Richard Roberts
 * @date    Feb 14, 2011
 */

#include <CppUnitLite/TestHarness.h>

#undef NDEBUG
#define NDEBUG
#undef GTSAM_ENABLE_DEBUG
#include <gtsam/base/debug.h>

/* ************************************************************************* */
TEST(Debug, debug_disabled) {
  const bool debug1 = ISDEBUG("TestDebug");
  EXPECT(!debug1);

  SETDEBUG("TestDebug", true);
  bool debug2 = ISDEBUG("TestDebug");
  EXPECT(!debug2);
}

#define GTSAM_ENABLE_DEBUG
#include <gtsam/base/debug.h>

/* ************************************************************************* */
TEST(Debug, debug_enabled) {
  const bool debug1 = ISDEBUG("TestDebug");
  EXPECT(!debug1);

  SETDEBUG("TestDebug", true);
  bool debug2 = ISDEBUG("TestDebug");
  EXPECT(debug2);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
