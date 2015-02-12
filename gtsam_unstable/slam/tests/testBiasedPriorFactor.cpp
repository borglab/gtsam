/**
 * @file   testBiasedPriorFactor.cpp
 * @brief  Test Biased Prior Factor
 * @author Siddharth Choudhary
 * @date   Feb 6, 2015
 */

#include <gtsam/geometry/Pose2.h>
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

// Check Jacobians for Pose2
TEST(BiasedPriorFactor, JacobianPose2) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.4);
  Pose2 z(1,1,0.5);
  Vector3 u(1,2,3);
  BiasedPriorFactor<Pose2, Vector3> factor(1, z, u, model);

  Pose2 x(0.5,0.5,0.1);
  Matrix Aactual;
  Vector bActual = factor.evaluateError(x,Aactual);

  Matrix Aexpected = eye(3);
  Pose2 zinv_x = z.between(x);
  Vector3 zinv_x_xyth = Vector3(zinv_x.x(),zinv_x.y(),zinv_x.theta());
  Vector bExpected = zinv_x_xyth + u;

  EXPECT(assert_equal(Aexpected, Aactual, 1e-9));
  EXPECT(assert_equal(bExpected, bActual, 1e-9));

  Vector bExpected2 = z.localCoordinates(x) + u;
  EXPECT(assert_equal(bExpected2, bActual, 1e-9));

  // The following does not work, as the "local", by default does not do logmap
  //  Vector bExpected3 = Pose2::Logmap(z.between(x)) + u;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

