/**
 * @file testVelocityConstraint
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/dynamics/VelocityConstraint.h>

using namespace gtsam;

const double tol=1e-5;

const Key x1 = 1, x2 = 2;
const double dt = 1.0;

PoseRTV origin,
        pose1(Point3(0.5, 0.0, 0.0), Rot3::identity(), Velocity3(1.0, 0.0, 0.0)),
        pose1a(Point3(0.5, 0.0, 0.0)),
        pose2(Point3(1.5, 0.0, 0.0), Rot3::identity(), Velocity3(1.0, 0.0, 0.0));

/* ************************************************************************* */
TEST( testVelocityConstraint, trapezoidal ) {
  // hard constraints don't need a noise model
  VelocityConstraint constraint(x1, x2, dynamics::TRAPEZOIDAL, dt);

  // verify error function
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, pose1), tol));
  EXPECT(assert_equal(Z_3x1, constraint.evaluateError(origin, origin), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*(-1.0), constraint.evaluateError(pose1, pose1), tol));
  EXPECT(assert_equal(Vector::Unit(3,0)*0.5, constraint.evaluateError(origin, pose1a), tol));
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
