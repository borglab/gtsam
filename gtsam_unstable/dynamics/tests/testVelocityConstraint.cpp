/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testVelocityConstraint
 * @author Alex Cunningham
 */

#include <gtsam/config.h>
#include <CppUnitLite/TestHarness.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/base/VectorConstants.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>

using namespace gtsam;

const double tol=1e-5;

const Key x1 = 1, x2 = 2;
const double dt = 1.0;

PoseRTV origin,
        pose1(Point3(0.5, 0.0, 0.0), Rot3(), Velocity3(1.0, 0.0, 0.0)),
        pose1a(Point3(0.5, 0.0, 0.0)),
        pose2(Point3(1.5, 0.0, 0.0), Rot3(), Velocity3(1.0, 0.0, 0.0));

/* ************************************************************************* */
TEST( testVelocityConstraint, trapezoidal ) {
  // hard constraints don't need a noise model
  VelocityConstraint constraint(x1, x2, dynamics::TRAPEZOIDAL, dt);

  // verify error function
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, pose1), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, origin), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*(-1.0), constraint.evaluateError(pose1, pose1), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*0.5, constraint.evaluateError(origin, pose1a), tol));

  // Fixed PoseRTV dimensions select BinaryJacobianFactor<3, 9, 9>, including
  // for this constrained model. Verify it preserves the old generic system.
  const Values values{{x1, genericValue(origin)}, {x2, genericValue(pose1)}};
  const auto generic = constraint.NoiseModelFactor::linearize(values);
  const auto specialized = constraint.linearize(values);
  const bool isBinary = static_cast<bool>(
      std::dynamic_pointer_cast<BinaryJacobianFactor<3, 9, 9>>(specialized));
  CHECK(isBinary);
  EXPECT(assert_equal(*generic, *specialized, tol));
}

/* ************************************************************************* */
TEST( testEulerVelocityConstraint, euler_start ) {
  // hard constraints create their own noise model
  VelocityConstraint constraint(x1, x2, dynamics::EULER_START, dt);

  // verify error function
  EXPECT(assert_equal(Vector::Unit(3,0)*0.5, constraint.evaluateError(origin, pose1), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, origin), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(pose1, pose2), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*0.5, constraint.evaluateError(origin, pose1a), tol));
}

/* ************************************************************************* */
TEST( testEulerVelocityConstraint, euler_end ) {
  // hard constraints create their own noise model
  VelocityConstraint constraint(x1, x2, dynamics::EULER_END, dt);

  // verify error function
  EXPECT(assert_equal(Vector::Unit(3,0)*(-0.5), constraint.evaluateError(origin, pose1), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, origin), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(pose1, pose2), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*0.5, constraint.evaluateError(origin, pose1a), tol));
}

/* ************************************************************************* */
#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
