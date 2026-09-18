/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include "ValuesCrossLibrary.h"

#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

#include <memory>
#include <typeinfo>

using values_cross_library::TestValue;

/* ************************************************************************* */
namespace cross_library_rtti {

// Verifies retrieval and update when GenericValue RTTI originates in a DSO.
TEST(Values, CrossLibraryGenericValue) {
  constexpr gtsam::Key key = 1;
  std::unique_ptr<gtsam::Value> foreignValue(
      values_cross_library::createValue(2.0));
  const gtsam::Value& foreignReference = *foreignValue;

  gtsam::Values values;
  values.insert(key, *foreignValue);
  values.update(key, TestValue(3.0));

  CHECK(typeid(foreignReference) == typeid(gtsam::GenericValue<TestValue>));
  DOUBLES_EQUAL(3.0, values.at<TestValue>(key).value(), 1e-9);

  const gtsam::Vector1 wrongType = gtsam::Vector1::Zero();
  CHECK_EXCEPTION(values.update(key, wrongType), gtsam::ValuesIncorrectType);
}

}  // namespace cross_library_rtti
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
/* ************************************************************************* */
