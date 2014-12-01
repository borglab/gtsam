/**
 * @file   testPriorFactor.cpp
 * @brief  Test PriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <gtsam/base/Vector.h>
#include <gtsam/slam/PriorFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

// Constructor scalar
TEST(PriorFactor, ConstructorScalar) {
  SharedNoiseModel model;
  PriorFactor<double> factor(1, 1.0, model);
}

// Constructor vector3
TEST(PriorFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  PriorFactor<Vector3> factor(1, Vector3(1,2,3), model);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
