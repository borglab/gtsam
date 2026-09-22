/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLinearExceptions.cpp
 * @brief Tests for exceptions thrown by linear solvers.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>

#include <string>

using namespace gtsam;

/* ************************************************************************* */
namespace indeterminate_system_exception {

// Verifies that the correctly named exception reports its nearby variable.
TEST(IndeterminateSystemException, NearbyVariableAndMessage) {
  const Key key = Symbol('x', 42);
  const IndeterminateSystemException exception(key);

  EXPECT_LONGS_EQUAL(key, exception.nearbyVariable());
  const std::string message = exception.what();
  CHECK(message.find("Indeterminate linear system") != std::string::npos);
  CHECK(message.find("Indeterminant linear system") == std::string::npos);
  CHECK(message.find("nearly indeterminate systems") != std::string::npos);
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
// Verifies that the 4.3 compatibility spelling catches the replacement type.
TEST(IndeterminateSystemException, DeprecatedAlias) {
  bool caught = false;
  try {
    throw IndeterminateSystemException(Symbol('l', 7));
  } catch (const IndeterminantLinearSystemException& exception) {
    caught = exception.nearbyVariable() == Symbol('l', 7);
  }
  CHECK(caught);
}
#endif

}  // namespace indeterminate_system_exception
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
