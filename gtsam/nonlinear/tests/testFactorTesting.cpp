/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testFactorTesting.cpp
 * @date September 18, 2014
 * @author Brice Rebsamen
 * @brief unit tests for testFactorJacobians and testExpressionJacobians
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/expressions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
Vector3 bodyVelocity(const Pose3& w_t_b,
                     const Vector3& vec_w,
                     OptionalJacobian<3, 6> Hpose = {},
                     OptionalJacobian<3, 3> Hvel = {}) {
  Matrix36 Hrot__pose;
  Rot3 w_R_b = w_t_b.rotation(Hrot__pose);
  Matrix33 Hvel__rot;
  Vector3 vec_b = w_R_b.unrotate(vec_w, Hvel__rot, Hvel);
  if (Hpose) {
    *Hpose = Hvel__rot * Hrot__pose;
  }
  return vec_b;
}

// Functor used to create an expression for the measured wheel speed scaled
// by the scale factor.
class ScaledVelocityFunctor {
 public:
  explicit ScaledVelocityFunctor(double measured_wheel_speed)
    : measured_velocity_(measured_wheel_speed, 0, 0) {}

  // Computes the scaled measured velocity vector from the measured wheel speed
  // and velocity scale factor. Also computes the corresponding jacobian
  // (w.r.t. the velocity scale).
  Vector3 operator()(double vscale,
                     OptionalJacobian<3, 1> H = {}) const {
    // The velocity scale factor value we are optimizing for is centered around
    // 0, so we need to add 1 to it before scaling the velocity.
    const Vector3 scaled_velocity = (vscale + 1.0) * measured_velocity_;
    if (H) {
      *H = measured_velocity_;
    }
    return scaled_velocity;
  }

 private:
  Vector3 measured_velocity_;
};

/* ************************************************************************* */
TEST(ExpressionTesting, Issue16) {
  const double tol = 1e-4;
  const double numerical_step = 1e-3;

  // Note: name of keys matters: if we use 'p' instead of 'x' then this no
  // longer repros the problem from issue 16. This is because the order of
  // evaluation in linearizeNumerically depends on the key values. To repro
  // we want to first evaluate the jacobian for the scale, then velocity,
  // then pose.
  const auto pose_key = Symbol('x', 1);
  const auto vel_key = Symbol('v', 1);
  const auto scale_key = Symbol('s', 1);

  Values values;
  values.insert<Pose3>(pose_key, Pose3());
  values.insert<Vector3>(vel_key, Vector3(1, 0, 0));
  values.insert<double>(scale_key, 0);

  const Vector3_ body_vel(&bodyVelocity,
                          Pose3_(pose_key),
                          Vector3_(vel_key));
  const Vector3_ scaled_measured_vel(ScaledVelocityFunctor(1),
                                     Double_(scale_key));
  const auto err_expr = body_vel - scaled_measured_vel;

  const auto err = err_expr.value(values);
  EXPECT_LONGS_EQUAL(3, err.size());
  EXPECT(assert_equal(Vector3(Z_3x1), err));
  EXPECT(internal::testExpressionJacobians(
      "ScaleAndCompare", err_expr, values, numerical_step, tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

