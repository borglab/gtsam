/**
 * @file testPendulumExplicitEuler.cpp
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/dynamics/Pendulum.h>

/* ************************************************************************* */
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

namespace {

  const double tol=1e-5;
  const double h = 0.1;
  const double g = 9.81, l = 1.0;

  const double deg2rad = M_PI/180.0;
  double q1(deg2rad*30.0), q2(deg2rad*31.0);
  double v1(deg2rad*1.0/h), v2((v1-h*g/l*sin(q1)));

}

/* ************************************************************************* */
TEST( testPendulumFactor1, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactor1 constraint(Q(2), Q(1), V(1), h);

  // verify error function
  EXPECT(assert_equal(Z_1x1, constraint.evaluateError(q2, q1, v1), tol));
}

/* ************************************************************************* */
TEST( testPendulumFactor2, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactor2 constraint(V(2), V(1), Q(1), h);

  // verify error function
  EXPECT(assert_equal(Z_1x1, constraint.evaluateError(v2, v1, q1), tol));
}

/* ************************************************************************* */
TEST( testPendulumFactorPk, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactorPk constraint(P(1), Q(1), Q(2), h);

  double pk( 1/h * (q2-q1) + h*g*sin(q1) );

  // verify error function
  EXPECT(assert_equal(Z_1x1, constraint.evaluateError(pk, q1, q2), tol));
}

/* ************************************************************************* */
TEST( testPendulumFactorPk1, evaluateError) {
  // hard constraints don't need a noise model
  PendulumFactorPk1 constraint(P(2), Q(1), Q(2), h);

  double pk1( 1/h * (q2-q1) );

  // verify error function
  EXPECT(assert_equal(Z_1x1, constraint.evaluateError(pk1, q1, q2), tol));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
