/**
 * @file testVelocityConstraint3
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/dynamics/VelocityConstraint3.h>

/* ************************************************************************* */
using namespace gtsam;

const double tol=1e-5;
const double dt = 1.0;

LieScalar origin,
        x1(1.0), x2(2.0), v(1.0);

/* ************************************************************************* */
TEST( testVelocityConstraint3, evaluateError) {
  // hard constraints don't need a noise model
  VelocityConstraint3 constraint(x1, x2, v, dt);

  // verify error function
  EXPECT(assert_equal(zero(1), constraint.evaluateError(x1, x2, v), tol));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
