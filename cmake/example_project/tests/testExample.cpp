/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     testExample.cpp
 * @brief    Unit tests for example
 * @author   Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>

#include <example/PrintExamples.h>

using namespace gtsam;

TEST(Example, HelloString) {
  const std::string expectedString = "Hello!";
  EXPECT(assert_equal(expectedString, example::internal::getHelloString()));
}

TEST(Example, GoodbyeString) {
  const std::string expectedString = "See you soon!";
  EXPECT(assert_equal(expectedString, example::internal::getGoodbyeString()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


