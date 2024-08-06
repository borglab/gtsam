/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRampFunction.h
 * @brief   unit tests for ramp functions
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constraint/RampFunction.h>

using namespace gtsam;

TEST(RampFunction, error_and_jacobian) {
  /// Helper function for numerical Jacobian computation.
  auto ramp_helper = [&](const double& x) { return RampFunction(x); };

  /// Create a set of values to test the function.
  static std::vector<double> x_vec{-3.0, 0.0, 1.0, 2.0, 3.0};
  static std::vector<double> expected_r_vec{0.0, 0.0, 1.0, 2.0, 3.0};

  for (size_t i = 0; i < x_vec.size(); i++) {
    double x = x_vec.at(i);
    Matrix H;
    double r = RampFunction(x, H);

    /// Check function evaluation.
    EXPECT_DOUBLES_EQUAL(expected_r_vec.at(i), r, 1e-9);

    /// Check derivative.
    if (abs(x) > 1e-6) {  // function is not smooth at 0, so Jacobian is undefined.
      Matrix expected_H = gtsam::numericalDerivative11<double, double, 1>(ramp_helper, x, 1e-6);
      EXPECT(assert_equal(expected_H, H));
    }
  }
}

TEST(RampFunctionPoly2, error_and_jacobian) {
  /// Helper function for numerical Jacobian computation.
  RampFunctionPoly2 p_ramp(2.0);
  auto ramp_helper = [&](const double& x) { return p_ramp(x); };

  /// Create a set of values to test the function.
  static std::vector<double> x_vec{-3.0, 0.0, 1.0, 2.0, 3.0};
  static std::vector<double> expected_r_vec{0.0, 0.0, 0.25, 1.0, 2.0};

  for (size_t i = 0; i < x_vec.size(); i++) {
    double x = x_vec.at(i);
    Matrix H;
    double r = p_ramp(x, H);

    /// Check function evaluation.
    EXPECT_DOUBLES_EQUAL(expected_r_vec.at(i), r, 1e-9);

    /// Check derivative.
    Matrix expected_H = gtsam::numericalDerivative11<double, double, 1>(ramp_helper, x, 1e-6);
    EXPECT(assert_equal(expected_H, H, 1e-6));
  }
}

TEST(RampFunctionPoly3, error_and_jacobian) {
  /// Helper function for numerical Jacobian computation.
  RampFunctionPoly3 p_ramp(2.0);
  auto ramp_helper = [&](const double& x) { return p_ramp(x); };

  /// Create a set of values to test the function.
  static std::vector<double> x_vec{-3.0, 0.0, 1.0, 2.0, 3.0};
  static std::vector<double> expected_r_vec{0.0, 0.0, 0.75, 2.0, 3.0};

  for (size_t i = 0; i < x_vec.size(); i++) {
    double x = x_vec.at(i);
    Matrix H;
    double r = p_ramp(x, H);

    /// Check function evaluation.
    EXPECT_DOUBLES_EQUAL(expected_r_vec.at(i), r, 1e-9);

    /// Check derivative.
    Matrix expected_H = gtsam::numericalDerivative11<double, double, 1>(ramp_helper, x, 1e-6);
    EXPECT(assert_equal(expected_H, H, 1e-6));
  }
}

TEST(SoftPlusFunction, error_and_jacobian) {
  /// Helper function for numerical Jacobian computation.
  SoftPlusFunction soft_plus(0.5);
  auto soft_plus_helper = [&](const double& x) { return soft_plus(x); };

  /// Create a set of values to test the function.
  static std::vector<double> x_vec{-3.0, 0.0, 1.0, 2.0, 3.0};
  static std::vector<double> expected_r_vec{
      0.40282656, 1.38629436, 1.94815397, 2.62652338, 3.40282656};

  for (size_t i = 0; i < x_vec.size(); i++) {
    double x = x_vec.at(i);
    Matrix H;
    double r = soft_plus(x, H);

    /// Check function evaluation.
    EXPECT_DOUBLES_EQUAL(expected_r_vec.at(i), r, 1e-6);

    /// Check derivative.
    Matrix expected_H = gtsam::numericalDerivative11<double, double, 1>(soft_plus_helper, x, 1e-6);
    EXPECT(assert_equal(expected_H, H, 1e-6));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
