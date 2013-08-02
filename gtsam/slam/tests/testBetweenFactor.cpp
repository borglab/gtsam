/**
 * @file 	 testBetweenFactor.cpp
 * @brief  
 * @author Duy-Nguyen Ta
 * @date 	 Aug 2, 2013
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace gtsam::noiseModel;

TEST(BetweenFactor, Rot3) {
  Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::rodriguez(0.4, 0.5, 0.6);
  Rot3 noise = Rot3::rodriguez(0.01, 0.01, 0.01);
  Rot3 measured = R1.between(R2)*noise  ;

  BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));
  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(R1, R2, actualH1, actualH2);

  Vector expected = Rot3::Logmap(measured.inverse() * R1.between(R2));
  CHECK(assert_equal(expected,actual, 1e-100));

  Matrix numericalH1 = numericalDerivative21(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
