/**
 * @file   testPriorFactor.cpp
 * @brief  Test PriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>
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
  Vector v(5);
  v << 1, 2, 3, 4, 5;
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

  ConstantBias incorrectBias(
      (Vector6() << 1.1, 2.1, 3.1, 0.2, 0.3, 0.4).finished());
  values.clear();
  values.insert(1, incorrectBias);
  EXPECT_DOUBLES_EQUAL(3.0, factor.error(values), 1e-8);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
