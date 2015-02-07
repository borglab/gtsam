/**
 * @file   testBiasedPriorFactor.cpp
 * @brief  Test Biased Prior Factor
 * @author Siddharth Choudhary
 * @date   Feb 6, 2015
 */

#include <gtsam/base/Vector.h>
#include <gtsam_unstable/slam/BiasedPriorFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

// Constructor scalar
TEST(BiasedPriorFactor, ConstructorScalar) {
  SharedNoiseModel model;
  Vector bias(1); bias << 1;
  BiasedPriorFactor<double, Vector> factor(1, 1.0, bias, model);
}

// Constructor vector3
TEST(BiasedPriorFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  BiasedPriorFactor<Vector3, Vector3> factor(1, Vector3(1,2,3), Vector3(1,2,3), model);
}

// Constructor dynamic sized vector
TEST(BiasedPriorFactor, ConstructorDynamicSizeVector) {
  Vector v(5); v << 1, 2, 3, 4, 5;
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  BiasedPriorFactor<Vector, Vector> factor(1, v, v, model);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

