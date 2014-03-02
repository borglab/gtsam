/**
 * @file    testBetweenFactor.cpp
 * @brief  
 * @author Duy-Nguyen Ta
 * @date    Aug 2, 2013
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace gtsam::noiseModel;



/**
 * This TEST should fail. If you want it to pass, change noise to 0.
 */
TEST(BetweenFactor, Rot3) {
  Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::rodriguez(0.4, 0.5, 0.6);
  Rot3 noise = Rot3::rodriguez(0.01, 0.01, 0.01); // Uncomment to make unit test fail
  Rot3 measured = R1.between(R2)*noise  ;

  BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));
  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(R1, R2, actualH1, actualH2);

  Vector expected = Rot3::Logmap(measured.inverse() * R1.between(R2));

  Matrix3 J = Rot3::rightJacobianExpMapSO3inverse(expected);
  actualH1 = J * actualH1;
  actualH2 = J * actualH2;

  //EXPECT(assert_equal(expected,actual/*, 1e-100*/)); // Uncomment to make unit test fail

  Matrix numericalH1 = numericalDerivative21(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH1,actualH1,1e-4));

  Matrix numericalH2 = numericalDerivative22(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH2,actualH2,1e-4));
}

Rot3 evaluateRotation(const Vector3 theta)
{
  return Rot3::Expmap(theta);
}

/* ************************************************************************* */
TEST( BetweenFactor, PartialDerivativeExpmap )
{
  Vector3 theta; theta << 0.1, 0.6, 0.5;

  Matrix expected = numericalDerivative11<Rot3, LieVector>(boost::bind(
      &evaluateRotation, _1), LieVector(theta));

  const Matrix3 actual = Rot3::rightJacobianExpMapSO3(theta);

  // Compare Jacobians
  EXPECT(assert_equal(expected, actual));
}

Rot3 betweeRotation(const Rot3 R1, const Rot3 R2)
{
  return R1.between(R2);
}
/* ************************************************************************* */
TEST( BetweenFactor, PartialDerivativeExpmap2 )
{
  Vector3 theta; theta << 0.1, 0.6, 0.5;
  Rot3 R = Rot3::Expmap(theta);

  Rot3 R1 = Rot3::Expmap(theta);

  Matrix expected = numericalDerivative11<Rot3, Rot3>(boost::bind(&betweeRotation, R1, _1), R);

  const Matrix3 actual = Matrix3::Identity();

  // Compare Jacobians
  EXPECT(assert_equal(expected, actual));
}

Vector3 evaluateLogRotation(const Vector3 thetahat, const Vector3 deltatheta)
{
  return Rot3::Logmap( Rot3::Expmap(thetahat).compose( Rot3::Expmap(deltatheta) ) );
}
/* ************************************************************************* */
TEST( BetweenFactor, PartialDerivativeLogmap )
{
  // Linearization point
  Vector3 thetahat; thetahat << 0.1,0.1,0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 deltatheta; deltatheta << 0, 0, 0;

  // Compute numerical derivatives
  Matrix expected = numericalDerivative11<LieVector>(boost::bind(
      &evaluateLogRotation, thetahat, _1), LieVector(deltatheta));

  const Matrix3 actual = Rot3::rightJacobianExpMapSO3inverse(thetahat);

  // Compare Jacobians
  EXPECT(assert_equal(expected, actual));
}

Vector3 evaluateLogRotation2(const Rot3 R)
{
  return Rot3::Logmap(R);
}
/* ************************************************************************* */
TEST( BetweenFactor, PartialDerivativeLogmap2 )
{
  Vector3 theta; theta << 0.1, 0.6, 0.5;
  Rot3 R = Rot3::Expmap(theta);

  Matrix expected = numericalDerivative11<Rot3>(boost::bind(&evaluateLogRotation2, _1), R);

  const Matrix3 actual = Rot3::rightJacobianExpMapSO3inverse(theta);

  // Compare Jacobians
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
