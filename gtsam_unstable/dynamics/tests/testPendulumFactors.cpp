/**
 * @file testPendulumExplicitEuler.cpp
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam_unstable/dynamics/Pendulum.h>

/* ************************************************************************* */
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

const double tol=1e-5;
const double dt = 0.1;
const double g = 9.81, l = 1.0;

const double deg2rad = M_PI/180.0;
LieScalar origin, q1(deg2rad*30.0), q2(deg2rad*31.0);
LieScalar v1(deg2rad*1.0/dt), v2((v1-dt*g/l*sin(q1)));

/* ************************************************************************* */
TEST( testPendulumFactor1, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactor1 constraint(Q(2), Q(1), V(1), dt);

  // verify error function
  EXPECT(assert_equal(zero(1), constraint.evaluateError(q2, q1, v1), tol));
}

/* ************************************************************************* */
TEST( testPendulumFactor2, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactor2 constraint(V(2), V(1), Q(1), dt);

  // verify error function
  EXPECT(assert_equal(zero(1), constraint.evaluateError(v2, v1, q1), tol));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
