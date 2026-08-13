/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include "ValuesCrossLibrary.h"

#include <gtsam/base/GenericValue.h>

#include <cmath>
#include <iostream>

namespace values_cross_library {

void TestValue::print(const std::string& label) const {
  std::cout << label << value_;
}

bool TestValue::equals(const TestValue& other, double tolerance) const {
  return std::abs(value_ - other.value_) <= tolerance;
}

size_t TestValue::dim() const { return dimension; }

TestValue TestValue::retract(const gtsam::Vector& delta) const {
  return TestValue(value_ + delta[0]);
}

gtsam::Vector TestValue::localCoordinates(const TestValue& other) const {
  return gtsam::Vector1(other.value_ - value_);
}

gtsam::Value* createValue(double value) {
  return new gtsam::GenericValue<TestValue>(TestValue(value));
}

}  // namespace values_cross_library
