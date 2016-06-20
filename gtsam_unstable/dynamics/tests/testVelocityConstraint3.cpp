/**
 * @file testVelocityConstraint3
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/dynamics/VelocityConstraint3.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;

/* ************************************************************************* */
// evaluateError
TEST( testVelocityConstraint3, evaluateError) {

  const double tol = 1e-5;
  const double dt = 1.0;
  double x1(1.0), x2(2.0), v(1.0);

  // hard constraints don't need a noise model
  VelocityConstraint3 constraint(X(1), X(2), V(1), dt);

  // verify error function
  EXPECT(assert_equal(Z_1x1, constraint.evaluateError(x1, x2, v), tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
