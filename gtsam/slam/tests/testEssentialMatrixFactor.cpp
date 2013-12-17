/*
 * @file testEssentialMatrixFactor.cpp
 * @brief Test EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/slam/EssentialMatrixFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>
#include <vector>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

//*************************************************************************
// Create two cameras and corresponding essential matrix E
Rot3 aRb = Rot3::yaw(M_PI_2);
Point3 aTb(0.1, 0, 0);
Pose3 identity, aPb(aRb, aTb);
typedef CalibratedCamera Cam;
Cam cameraA(identity), cameraB(aPb);
Matrix aEb_matrix = skewSymmetric(aTb.x(), aTb.y(), aTb.z()) * aRb.matrix();

// Create test data, we need at least 5 points
Point3 P[5] = { Point3(0, 0, 1), Point3(-0.1, 0, 1), Point3(0.1, 0, 1), //
Point3(0, 0.5, 0.5), Point3(0, -0.5, 0.5) };

// Project points in both cameras
vector<Point2> pA(5), pB(5);
vector<Point2>::iterator //
it1 = std::transform(P, P + 5, pA.begin(),
    boost::bind(&Cam::project, &cameraA, _1, boost::none, boost::none)), //
it2 = std::transform(P, P + 5, pB.begin(),
    boost::bind(&Cam::project, &cameraB, _1, boost::none, boost::none));

// Converto to homogenous coordinates
vector<Vector> vA(5), vB(5);
vector<Vector>::iterator //
it3 = std::transform(pA.begin(), pA.end(), vA.begin(),
    &EssentialMatrix::Homogeneous), //
it4 = std::transform(pB.begin(), pB.end(), vB.begin(),
    &EssentialMatrix::Homogeneous);

//*************************************************************************
TEST (EssentialMatrix, testData) {
  // Check E matrix
  Matrix expected(3, 3);
  expected << 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0;
  EXPECT(assert_equal(expected, aEb_matrix));

  // Check some projections
  EXPECT(assert_equal(Point2(0,0),pA[0]));
  EXPECT(assert_equal(Point2(0,0.1),pB[0]));
  EXPECT(assert_equal(Point2(0,-1),pA[4]));
  EXPECT(assert_equal(Point2(-1,0.2),pB[4]));

  // Check homogeneous version
  EXPECT(assert_equal((Vector(3) << -1,0.2,1),vB[4]));

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA[i].transpose() * aEb_matrix * vB[i], 1e-8);

  // Check epipolar constraint
  EssentialMatrix trueE(aRb, aTb);
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA[i],vB[i]), 1e-8);
}

//*************************************************************************
TEST (EssentialMatrix, factor) {
  EssentialMatrix trueE(aRb, aTb);
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(1);

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor factor(1, pA[i], pB[i], model);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix HActual;
    Vector actual = factor.evaluateError(trueE, HActual);
    EXPECT(assert_equal(expected, actual, 1e-8));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix HExpected;
    HExpected = numericalDerivative11<EssentialMatrix>(
        boost::bind(&EssentialMatrixFactor::evaluateError, &factor, _1,
            boost::none), trueE);

    // Verify the Jacobian is correct
    CHECK(assert_equal(HExpected, HActual, 1e-9));
  }
}

//*************************************************************************
TEST (EssentialMatrix, fromConstraints) {
  // Here we want to optimize directly on essential matrix constraints
  // Yi Ma's algorithm (Ma01ijcv) is a bit cumbersome to implement,
  // but GTSAM does the equivalent anyway, provided we give the right
  // factors. In this case, the factors are the constraints.

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,0.01);
  for (size_t i = 0; i < 5; i++)
    graph.add(EssentialMatrixFactor(1, pA[i], pB[i], model));

  // Check error at ground truth
  Values truth;
  EssentialMatrix trueE(aRb, aTb);
  truth.insert(1, trueE);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE = trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1));
  initial.insert(1, initialE);
  EXPECT_DOUBLES_EQUAL(640, graph.error(initial), 1e-2);

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(1);
  EXPECT(assert_equal(trueE, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, actual.error(vA[i],vB[i]), 1e-6);

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

