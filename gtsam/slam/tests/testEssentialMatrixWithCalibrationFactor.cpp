/**
 * @file testEssentialMatrixWithCalibrationFactor.cpp
 * @brief Test EssentialMatrixWithCalibrationFactor class
 * @author Ayush Baid
 * @author Akshay Krishnan
 * @date April 22, 2021
 */

#include <gtsam/slam/EssentialMatrixWithCalibrationFactor.h>

#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// Noise model for first type of factor is evaluating algebraic error
noiseModel::Isotropic::shared_ptr model1 = noiseModel::Isotropic::Sigma(1,
    0.01);
// Noise model for second type of factor is evaluating pixel coordinates
noiseModel::Unit::shared_ptr model2 = noiseModel::Unit::Create(2);

// The rotation between body and camera is:
gtsam::Point3 bX(1, 0, 0), bY(0, 1, 0), bZ(0, 0, 1);
gtsam::Rot3 cRb = gtsam::Rot3(bX, bZ, -bY).inverse();

namespace example1 {

const string filename = findExampleDataFile("5pointExample1.txt");
SfmData data;
bool readOK = readBAL(filename, data);
Rot3 c1Rc2 = data.cameras[1].pose().rotation();
Point3 c1Tc2 = data.cameras[1].pose().translation();
Cal3Bundler trueK = data.cameras[1].calibration(); // TODO: maybe default value not good; assert with 0th
// PinholeCamera<Cal3Bundler> camera2(data.cameras[1].pose(), trueK);
Rot3 trueRotation(c1Rc2);
Unit3 trueDirection(c1Tc2);
EssentialMatrix trueE(trueRotation, trueDirection);

double baseline = 0.1; // actual baseline of the camera

Point2 pA(size_t i) {
  return data.tracks[i].measurements[0].second;
}
Point2 pB(size_t i) {
  return data.tracks[i].measurements[1].second;
}
Vector vA(size_t i, Cal3Bundler K) {
  return EssentialMatrix::Homogeneous(K.calibrate(pA(i)));
}
Vector vB(size_t i, Cal3Bundler K) {
  return EssentialMatrix::Homogeneous(K.calibrate(pB(i)));
}

//*************************************************************************
TEST (EssentialMatrixWithCalibrationFactor, testData) {
  CHECK(readOK);

  // Check E matrix
  Matrix expected(3, 3);
  expected << 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0;
  Matrix aEb_matrix = skewSymmetric(c1Tc2.x(), c1Tc2.y(), c1Tc2.z())
      * c1Rc2.matrix();
  EXPECT(assert_equal(expected, aEb_matrix, 1e-8));

  // Check some projections
  EXPECT(assert_equal(Point2(0, 0), pA(0), 1e-8));
  EXPECT(assert_equal(Point2(0, 0.1), pB(0), 1e-8));
  EXPECT(assert_equal(Point2(0, -1), pA(4), 1e-8));
  EXPECT(assert_equal(Point2(-1, 0.2), pB(4), 1e-8));

  // Check homogeneous version
  EXPECT(assert_equal(Vector3(-1, 0.2, 1), vB(4, trueK), 1e-8));

  // check the calibration
  Cal3Bundler expectedK;
  EXPECT(assert_equal(expectedK, trueK));


  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA(i, trueK).transpose() * aEb_matrix * vB(i, trueK), 1e-8);

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA(i, trueK), vB(i, trueK)), 1e-7);
}

//*************************************************************************
TEST (EssentialMatrixWithCalibrationFactor, factor) {
  Key keyE(1);
  Key keyK(1);
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixWithCalibrationFactor<Cal3Bundler> factor(keyE, keyK, pA(i), pB(i), model1);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix HEactual;
    Matrix HKactual;
    Vector actual = factor.evaluateError(trueE, trueK, HEactual, HKactual);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix HEexpected;
    Matrix HKexpected;
    typedef Eigen::Matrix<double,1,1> Vector1;
    // TODO: fix this
    boost::function<Vector(const EssentialMatrix&, const Cal3Bundler&)> f = boost::bind(
      &EssentialMatrixWithCalibrationFactor<Cal3Bundler>::evaluateError, factor, _1, _2, boost::none, boost::none);
    HEexpected = numericalDerivative21<Vector1, EssentialMatrix, Cal3Bundler>(f, trueE, trueK);
    HKexpected = numericalDerivative22<Vector1, EssentialMatrix, Cal3Bundler>(f, trueE, trueK);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(HEexpected, HEactual, 1e-8));
    EXPECT(assert_equal(HKexpected, HKactual, 1e-5));
  }
}

// //*************************************************************************
// TEST(EssentialMatrixWithCalibrationFactor, ExpressionFactor) {
//   Key keyE(1);
//   Key keyK(2);
//   for (size_t i = 0; i < 5; i++) {
//     boost::function<double(const EssentialMatrix&, const Cal3Bundler&,
//       OptionalJacobian<1, 5>, OptionalJacobian<1, 3>)> f =
//       boost::bind(&EssentialMatrix::error, _1, pA(i), pB(i), _2);
//     Expression<EssentialMatrix> E_(keyE); // leaf expression
//     Expression<Cal3Bundler> K_(keyK); // leaf expression
//     Expression<double> expr(f, E_, K_); // unary expression

//     // Test the derivatives using Paul's magic
//     Values values;
//     values.insert(keyE, trueE);
//     values.insert(keyK, trueK);
//     EXPECT_CORRECT_EXPRESSION_JACOBIANS(expr, values, 1e-5, 1e-9);

//     // Create the factor
//     ExpressionFactor<double> factor(model1, 0, expr);

//     // Check evaluation
//     Vector expected(1);
//     expected << 0;
//     vector<Matrix> Hactual(1);
//     Vector actual = factor.unwhitenedError(values, Hactual);
//     EXPECT(assert_equal(expected, actual, 1e-7));
//   }
// }

//*************************************************************************
// TEST(EssentialMatrixWithCalibrationFactor, ExpressionFactorRotationOnly) {
//   Key keyE(1);
//   Key keyK(1);
//   for (size_t i = 0; i < 5; i++) {
//     boost::function<double(const EssentialMatrix&, OptionalJacobian<1, 5>)> f =
//         boost::bind(&EssentialMatrix::error, _1, vA(i), vB(i), _2);
//     boost::function<EssentialMatrix(const Rot3&, const Unit3&, OptionalJacobian<5, 3>,
//                                     OptionalJacobian<5, 2>)> g;
//     Expression<Rot3> R_(key);
//     Expression<Unit3> d_(trueDirection);
//     Expression<EssentialMatrix> E_(&EssentialMatrix::FromRotationAndDirection, R_, d_);
//     Expression<double> expr(f, E_);

//     // Test the derivatives using Paul's magic
//     Values values;
//     values.insert(key, trueRotation);
//     EXPECT_CORRECT_EXPRESSION_JACOBIANS(expr, values, 1e-5, 1e-9);

//     // Create the factor
//     ExpressionFactor<double> factor(model1, 0, expr);

//     // Check evaluation
//     Vector expected(1);
//     expected << 0;
//     vector<Matrix> Hactual(1);
//     Vector actual = factor.unwhitenedError(values, Hactual);
//     EXPECT(assert_equal(expected, actual, 1e-7));
//   }
// }

//*************************************************************************
TEST(EssentialMatrixWithCalibrationFactor, evaluateErrorJacobians) {
  Key keyE(1);
  Key keyK(2);
  // initialize essential matrix
  Rot3 r = Rot3::Expmap(Vector3(M_PI/6, M_PI / 3, M_PI/9));
  Unit3 t(Point3(2, -1, 0.5));
  EssentialMatrix E = EssentialMatrix::FromRotationAndDirection(r, t);
  Cal3_S2 K(200, 1, 1, 10, 10);
  Values val;
  val.insert(keyE, E);
  val.insert(keyK, K);

  Point2 pA(10.0, 20.0);
  Point2 pB(12.0, 15.0);

  EssentialMatrixWithCalibrationFactor<Cal3_S2> f(keyE, keyK, pA, pB, model1);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-6);
}

//*************************************************************************
TEST (EssentialMatrixWithCalibrationFactor, minimization) {
  // Here we want to optimize directly on essential matrix constraints
  // Yi Ma's algorithm (Ma01ijcv) is a bit cumbersome to implement,
  // but GTSAM does the equivalent anyway, provided we give the right
  // factors. In this case, the factors are the constraints.

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixWithCalibrationFactor<Cal3Bundler>>(1, 2, pA(i), pB(i), model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE = trueE.retract(
      (Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3Bundler initialK = trueK.retract(
      (Vector(3) << 0.1, -1e-1, 3e-2).finished());
  initial.insert(1, initialE);
  initial.insert(2, initialK);
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(618.94, graph.error(initial), 1e-2);
#else
  EXPECT_DOUBLES_EQUAL(639.84, graph.error(initial), 1e-2);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3Bundler actualK = result.at<Cal3Bundler>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1));
  EXPECT(assert_equal(trueK, actualK, 1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, actualE.error(vA(i, actualK), vB(i, actualK)), 1e-6);

}

} // namespace example1

//*************************************************************************

// namespace example2 {

// const string filename = findExampleDataFile("5pointExample2.txt");
// SfmData data;
// bool readOK = readBAL(filename, data);
// Rot3 aRb = data.cameras[1].pose().rotation();
// Point3 aTb = data.cameras[1].pose().translation();
// EssentialMatrix trueE(aRb, Unit3(aTb));

// double baseline = 10; // actual baseline of the camera

// Point2 pA(size_t i) {
//   return data.tracks[i].measurements[0].second;
// }
// Point2 pB(size_t i) {
//   return data.tracks[i].measurements[1].second;
// }

// boost::shared_ptr<Cal3Bundler> //
// K = boost::make_shared<Cal3Bundler>(500, 0, 0);
// PinholeCamera<Cal3Bundler> camera2(data.cameras[1].pose(), *K);

// Vector vA(size_t i) {
//   Point2 xy = K->calibrate(pA(i));
//   return EssentialMatrix::Homogeneous(xy);
// }
// Vector vB(size_t i) {
//   Point2 xy = K->calibrate(pB(i));
//   return EssentialMatrix::Homogeneous(xy);
// }

// //*************************************************************************
// TEST (EssentialWithMatrixCalibrationFactor, extraMinimization) {
//   // Additional test with camera moving in positive X direction

//   NonlinearFactorGraph graph;
//   for (size_t i = 0; i < 5; i++)
//     graph.emplace_shared<EssentialMatrixWithCalibrationFactor>(1, pA(i), pB(i), model1, K);

//   // Check error at ground truth
//   Values truth;
//   truth.insert(1, trueE);
//   EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

//   // Check error at initial estimate
//   Values initial;
//   EssentialMatrix initialE = trueE.retract(
//       (Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
//   initial.insert(1, initialE);

// #if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
//   EXPECT_DOUBLES_EQUAL(643.26, graph.error(initial), 1e-2);
// #else
//   EXPECT_DOUBLES_EQUAL(639.84, graph.error(initial), 1e-2);
// #endif

//   // Optimize
//   LevenbergMarquardtParams parameters;
//   LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
//   Values result = optimizer.optimize();

//   // Check result
//   EssentialMatrix actual = result.at<EssentialMatrix>(1);
//   EXPECT(assert_equal(trueE, actual, 1e-1));

//   // Check error at result
//   EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

//   // Check errors individually
//   for (size_t i = 0; i < 5; i++)
//     EXPECT_DOUBLES_EQUAL(0, actual.error(vA(i), vB(i)), 1e-6);

// }

// //*************************************************************************
// TEST (EssentialMatrixFactor2, extraTest) {
//   for (size_t i = 0; i < 5; i++) {
//     EssentialMatrixFactor2 factor(100, i, pA(i), pB(i), model2, K);

//     // Check evaluation
//     Point3 P1 = data.tracks[i].p;
//     const Point2 pi = camera2.project(P1);
//     Point2 expected(pi - pB(i));

//     Matrix Hactual1, Hactual2;
//     double d(baseline / P1.z());
//     Vector actual = factor.evaluateError(trueE, d, Hactual1, Hactual2);
//     EXPECT(assert_equal(expected, actual, 1e-7));

//     // Use numerical derivatives to calculate the expected Jacobian
//     Matrix Hexpected1, Hexpected2;
//     boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
//         &EssentialMatrixFactor2::evaluateError, &factor, _1, _2, boost::none,
//         boost::none);
//     Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, trueE, d);
//     Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, trueE, d);

//     // Verify the Jacobian is correct
//     EXPECT(assert_equal(Hexpected1, Hactual1, 1e-6));
//     EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
//   }
// }

// //*************************************************************************
// TEST (EssentialMatrixFactor2, extraMinimization) {
//   // Additional test with camera moving in positive X direction

//   // We start with a factor graph and add constraints to it
//   // Noise sigma is 1, assuming pixel measurements
//   NonlinearFactorGraph graph;
//   for (size_t i = 0; i < data.number_tracks(); i++)
//     graph.emplace_shared<EssentialMatrixFactor2>(100, i, pA(i), pB(i), model2, K);

//   // Check error at ground truth
//   Values truth;
//   truth.insert(100, trueE);
//   for (size_t i = 0; i < data.number_tracks(); i++) {
//     Point3 P1 = data.tracks[i].p;
//     truth.insert(i, double(baseline / P1.z()));
//   }
//   EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

//   // Optimize
//   LevenbergMarquardtParams parameters;
//   // parameters.setVerbosity("ERROR");
//   LevenbergMarquardtOptimizer optimizer(graph, truth, parameters);
//   Values result = optimizer.optimize();

//   // Check result
//   EssentialMatrix actual = result.at<EssentialMatrix>(100);
//   EXPECT(assert_equal(trueE, actual, 1e-1));
//   for (size_t i = 0; i < data.number_tracks(); i++)
//     EXPECT_DOUBLES_EQUAL(truth.at<double>(i), result.at<double>(i), 1e-1);

//   // Check error at result
//   EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
// }

// //*************************************************************************
// TEST (EssentialMatrixFactor3, extraTest) {

//   // The "true E" in the body frame is
//   EssentialMatrix bodyE = cRb.inverse() * trueE;

//   for (size_t i = 0; i < 5; i++) {
//     EssentialMatrixFactor3 factor(100, i, pA(i), pB(i), cRb, model2, K);

//     // Check evaluation
//     Point3 P1 = data.tracks[i].p;
//     const Point2 pi = camera2.project(P1);
//     Point2 expected(pi - pB(i));

//     Matrix Hactual1, Hactual2;
//     double d(baseline / P1.z());
//     Vector actual = factor.evaluateError(bodyE, d, Hactual1, Hactual2);
//     EXPECT(assert_equal(expected, actual, 1e-7));

//     // Use numerical derivatives to calculate the expected Jacobian
//     Matrix Hexpected1, Hexpected2;
//     boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
//         &EssentialMatrixFactor3::evaluateError, &factor, _1, _2, boost::none,
//         boost::none);
//     Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, bodyE, d);
//     Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, bodyE, d);

//     // Verify the Jacobian is correct
//     EXPECT(assert_equal(Hexpected1, Hactual1, 1e-6));
//     EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
//   }
// }

// } // namespace example2

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
