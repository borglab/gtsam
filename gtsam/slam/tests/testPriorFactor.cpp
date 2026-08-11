/**
 * @file   testPriorFactor.cpp
 * @brief  Test PriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace std;
using namespace std::placeholders;
using namespace gtsam;
using namespace imuBias;

/* ************************************************************************* */

// Constructor scalar
TEST(PriorFactor, ConstructorScalar) {
  SharedNoiseModel model;
  PriorFactor<double> factor(1, 1.0, model);
}

// Constructor vector3
TEST(PriorFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  PriorFactor<Vector3> factor(1, Vector3(1, 2, 3), model);
}

// Constructor dynamic sized vector
TEST(PriorFactor, ConstructorDynamicSizeVector) {
  Vector v{{1, 2, 3, 4, 5}};
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  PriorFactor<Vector> factor(1, v, model);
}

Vector callEvaluateError(const PriorFactor<ConstantBias>& factor,
                         const ConstantBias& bias) {
  return factor.evaluateError(bias);
}

// Test for imuBias::ConstantBias
TEST(PriorFactor, ConstantBias) {
  Vector3 biasAcc(1, 2, 3);
  Vector3 biasGyro(0.1, 0.2, 0.3);
  ConstantBias bias(biasAcc, biasGyro);

  PriorFactor<ConstantBias> factor(1, bias,
                                   noiseModel::Isotropic::Sigma(6, 0.1));
  Values values;
  values.insert(1, bias);

  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-8);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  ConstantBias incorrectBias(Vector6{1.1, 2.1, 3.1, 0.2, 0.3, 0.4});
  values.clear();
  values.insert(1, incorrectBias);
  EXPECT_DOUBLES_EQUAL(3.0, factor.error(values), 1e-8);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
// Verifies the corrected Local Jacobian at a nonzero residual.
TEST(PriorFactor, Pose2Jacobians) {
  const Pose2 prior(1.0, 2.0, 0.3);
  PriorFactor<Pose2> factor(1, prior,
                            noiseModel::Isotropic::Sigma(3, 0.1));

  Values values;
  values.insert(1, Pose2(1.2, 1.9, 0.5));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}
#endif

// Verifies the identity fallback when Local Jacobians are unavailable.
TEST(PriorFactor, Similarity3WithoutLocalJacobians) {
  const Similarity3 prior;
  PriorFactor<Similarity3> factor(
      1, prior, noiseModel::Isotropic::Sigma(7, 0.1));
  const Similarity3 value =
      Similarity3::Expmap(Vector7{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.1});

  Matrix actualH;
  factor.evaluateError(value, actualH);
  EXPECT(assert_equal(Matrix7::Identity(), actualH));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
