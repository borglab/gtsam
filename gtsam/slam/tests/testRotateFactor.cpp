/*
 * @file testRotateFactor.cpp
 * @brief Test RotateFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/slam/RotateFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <vector>

using namespace std;
using namespace std::placeholders;
using namespace gtsam;

static const double kDegree = M_PI / 180;

//*************************************************************************
// Create some test data
// Let's assume IMU is aligned with aero (X-forward,Z down)
// And camera is looking forward.
static const Point3 cameraX(0, 1, 0), cameraY(0, 0, 1), cameraZ(1, 0, 0);
static const Rot3 iRc(cameraX, cameraY, cameraZ);

// Now, let's create some rotations around IMU frame
static const Unit3 p1(1, 0, 0), p2(0, 1, 0), p3(0, 0, 1);
static const Rot3 i1Ri2 = Rot3::AxisAngle(p1, 1), //
i2Ri3 = Rot3::AxisAngle(p2, 1), //
i3Ri4 = Rot3::AxisAngle(p3, 1);

// The corresponding rotations in the camera frame
static const Rot3 c1Zc2 = iRc.inverse() * i1Ri2 * iRc, //
c2Zc3 = iRc.inverse() * i2Ri3 * iRc, //
c3Zc4 = iRc.inverse() * i3Ri4 * iRc;

// The corresponding rotated directions in the camera frame
static const Unit3 z1 = iRc.inverse() * p1, //
z2 = iRc.inverse() * p2, //
z3 = iRc.inverse() * p3;

typedef noiseModel::Isotropic::shared_ptr Model;

//*************************************************************************
TEST (RotateFactor, checkMath) {
  EXPECT(assert_equal(c1Zc2, Rot3::AxisAngle(z1, 1)));
  EXPECT(assert_equal(c2Zc3, Rot3::AxisAngle(z2, 1)));
  EXPECT(assert_equal(c3Zc4, Rot3::AxisAngle(z3, 1)));
}

//*************************************************************************
TEST (RotateFactor, test) {
  Model model = noiseModel::Isotropic::Sigma(3, 0.01);
  RotateFactor f(1, i1Ri2, c1Zc2, model);
  EXPECT(assert_equal(Z_3x1, f.evaluateError(iRc), 1e-8));

  Rot3 R = iRc.retract(Vector3(0.1, 0.2, 0.1));
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  Vector expectedE = Vector3(-0.0248752, 0.202981, -0.0890529);
#else
  Vector expectedE = Vector3(-0.0246305, 0.20197, -0.08867);
#endif
  EXPECT( assert_equal(expectedE, f.evaluateError(R), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Vector3, Rot3>(
			[&f](const Rot3& r) { return f.evaluateError(r); }, iRc);
    f.evaluateError(iRc, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Vector3, Rot3>(
			[&f](const Rot3& r) { return f.evaluateError(r); }, R);
    f.evaluateError(R, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
}

//*************************************************************************
TEST (RotateFactor, minimization) {
  // Let's try to recover the correct iRc by minimizing
  NonlinearFactorGraph graph;
  Model model = noiseModel::Isotropic::Sigma(3, 0.01);
  graph.emplace_shared<RotateFactor>(1, i1Ri2, c1Zc2, model);
  graph.emplace_shared<RotateFactor>(1, i2Ri3, c2Zc3, model);
  graph.emplace_shared<RotateFactor>(1, i3Ri4, c3Zc4, model);

  // Check error at ground truth
  Values truth;
  truth.insert(1, iRc);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  Rot3 initialE = iRc.retract(kDegree * Vector3(20, -20, 20));
  initial.insert(1, initialE);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(3545.40, graph.error(initial), 1);
#else
  EXPECT_DOUBLES_EQUAL(3349, graph.error(initial), 1);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  //parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  Rot3 actual = result.at<Rot3>(1);
  EXPECT(assert_equal(iRc, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST (RotateDirectionsFactor, test) {
  Model model = noiseModel::Isotropic::Sigma(2, 0.01);
  RotateDirectionsFactor f(1, p1, z1, model);
  EXPECT(assert_equal(Z_2x1, f.evaluateError(iRc), 1e-8));

  Rot3 R = iRc.retract(Vector3(0.1, 0.2, 0.1));

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  Vector expectedE = Vector2(-0.0890529, -0.202981);
#else
  Vector expectedE = Vector2(-0.08867, -0.20197);
#endif

  EXPECT( assert_equal(expectedE, f.evaluateError(R), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Vector,Rot3>(
			[&f](const Rot3& r) {return f.evaluateError(r);}, iRc);
    f.evaluateError(iRc, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Vector,Rot3>(
			[&f](const Rot3& r) {return f.evaluateError(r);}, R);
    f.evaluateError(R, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
}

//*************************************************************************
TEST (RotateDirectionsFactor, minimization) {
  // Let's try to recover the correct iRc by minimizing
  NonlinearFactorGraph graph;
  Model model = noiseModel::Isotropic::Sigma(2, 0.01);
  graph.emplace_shared<RotateDirectionsFactor>(1, p1, z1, model);
  graph.emplace_shared<RotateDirectionsFactor>(1, p2, z2, model);
  graph.emplace_shared<RotateDirectionsFactor>(1, p3, z3, model);

  // Check error at ground truth
  Values truth;
  truth.insert(1, iRc);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  Rot3 initialE = iRc.retract(kDegree * Vector3(20, -20, 20));
  initial.insert(1, initialE);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(3335.9, graph.error(initial), 1);
#else
  EXPECT_DOUBLES_EQUAL(3162, graph.error(initial), 1);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  Rot3 actual = result.at<Rot3>(1);
  EXPECT(assert_equal(iRc, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST(RotateDirectionsFactor, Initialization) {
  // Create a gravity vector in a nav frame that has Z up
  const Point3 n_gravity(0, 0, -10);
  const Unit3 n_p(-n_gravity);

  // NOTE(frank): avoid singularities by using 85/275 instead of 90/270
  std::vector<double> angles = {0, 45, 85, 135, 180, 225, 275, 315};
  for (double yaw : angles) {
    const Rot3 nRy = Rot3::Yaw(yaw * kDegree);
    for (double pitch : angles) {
      const Rot3 yRp = Rot3::Pitch(pitch * kDegree);
      for (double roll : angles) {
        const Rot3 pRb = Rot3::Roll(roll * kDegree);

        // Rotation from body to nav frame:
        const Rot3 nRb = nRy * yRp * pRb;
        const Vector3 rpy = nRb.rpy() / kDegree;

        // Simulate measurement of IMU in body frame:
        const Point3 b_acc = nRb.unrotate(-n_gravity);
        const Unit3 b_z(b_acc);

        // Check initialization
        const Rot3 actual_nRb = RotateDirectionsFactor::Initialize(n_p, b_z);
        const Vector3 actual_rpy = actual_nRb.rpy() / kDegree;
        EXPECT_DOUBLES_EQUAL(rpy.x(), actual_rpy.x(), 1e-5);
        EXPECT_DOUBLES_EQUAL(rpy.y(), actual_rpy.y(), 1e-5);
      }
    }
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

