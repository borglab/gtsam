/**
 * @file testPendulumExplicitEuler.cpp
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam_unstable/dynamics/SimpleHelicopter.h>

/* ************************************************************************* */
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

const double tol=1e-5;
const double h = 1.0;
const double deg2rad = M_PI/180.0;

Pose3 g1(Rot3::ypr(deg2rad*10.0, deg2rad*20.0, deg2rad*30.0), Point3(100.0, 200.0, 300.0));
LieVector v1(Vector_(6, 0.1, 0.05, 0.02, 10.0, 20.0, 30.0));
Pose3 g2(g1.retract(h*v1, Pose3::EXPMAP));

/* ************************************************************************* */
Vector testExpmapDeriv(const LieVector& v) {
  return Pose3::Logmap(Pose3::Expmap(-v1)*Pose3::Expmap(v1+v));
}

TEST(Reconstruction, ExpmapInvDeriv) {
  Matrix numericalExpmap = numericalDerivative11(
      boost::function<Vector(const LieVector&)>(
          boost::bind(testExpmapDeriv,  _1)
          ),
      LieVector(Vector::Zero(6)), 1e-3
      );
  Matrix dExpInv = Pose3::dExpInv_TLN(v1);
  EXPECT(assert_equal(numericalExpmap, dExpInv, 5e-1));
}

/* ************************************************************************* */
TEST( Reconstruction, evaluateError) {
  // hard constraints don't need a noise model
  Reconstruction constraint(G(2), G(1), V(1), h);

  // verify error function
  Matrix H1, H2, H3;
  EXPECT(assert_equal(zero(6), constraint.evaluateError(g2, g1, v1, H1, H2, H3), tol));


  Matrix numericalH1 = numericalDerivative31(
      boost::function<Vector(const Pose3&, const Pose3&, const LieVector&)>(
          boost::bind(&Reconstruction::evaluateError, constraint, _1, _2, _3, boost::none, boost::none, boost::none)
          ),
      g2, g1, v1, 1e-5
      );

  Matrix numericalH2 = numericalDerivative32(
      boost::function<Vector(const Pose3&, const Pose3&, const LieVector&)>(
          boost::bind(&Reconstruction::evaluateError, constraint, _1, _2, _3, boost::none, boost::none, boost::none)
          ),
      g2, g1, v1, 1e-5
      );

  Matrix numericalH3 = numericalDerivative33(
      boost::function<Vector(const Pose3&, const Pose3&, const LieVector&)>(
          boost::bind(&Reconstruction::evaluateError, constraint, _1, _2, _3, boost::none, boost::none, boost::none)
          ),
      g2, g1, v1, 1e-5
      );

  EXPECT(assert_equal(numericalH1,H1,1e-5));
  EXPECT(assert_equal(numericalH2,H2,1e-5));
  EXPECT(assert_equal(numericalH3,H3,5e-1));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
