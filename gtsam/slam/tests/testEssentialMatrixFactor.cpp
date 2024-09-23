/*
 * @file testEssentialMatrixFactor.cpp
 * @brief Test EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/EssentialMatrixFactor.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/dataset.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Noise model for first type of factor is evaluating algebraic error
noiseModel::Isotropic::shared_ptr model1 =
    noiseModel::Isotropic::Sigma(1, 0.01);
// Noise model for second type of factor is evaluating pixel coordinates
noiseModel::Unit::shared_ptr model2 = noiseModel::Unit::Create(2);

// The rotation between body and camera is:
gtsam::Point3 bX(1, 0, 0), bY(0, 1, 0), bZ(0, 0, 1);
gtsam::Rot3 cRb = gtsam::Rot3(bX, bZ, -bY).inverse();

namespace example1 {

const string filename = findExampleDataFile("18pointExample1.txt");
SfmData data = SfmData::FromBalFile(filename);
Rot3 c1Rc2 = data.cameras[1].pose().rotation();
Point3 c1Tc2 = data.cameras[1].pose().translation();
// TODO: maybe default value not good; assert with 0th
Cal3_S2 trueK = Cal3_S2();
PinholeCamera<Cal3_S2> camera2(data.cameras[1].pose(), trueK);
Rot3 trueRotation(c1Rc2);
Unit3 trueDirection(c1Tc2);
EssentialMatrix trueE(trueRotation, trueDirection);
double baseline = 0.1;  // actual baseline of the camera

Point2 pA(size_t i) { return data.tracks[i].measurements[0].second; }
Point2 pB(size_t i) { return data.tracks[i].measurements[1].second; }
Vector vA(size_t i) { return EssentialMatrix::Homogeneous(pA(i)); }
Vector vB(size_t i) { return EssentialMatrix::Homogeneous(pB(i)); }

//*************************************************************************
TEST(EssentialMatrixFactor, testData) {
  // Check E matrix
  Matrix expected(3, 3);
  expected << 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0;
  Matrix aEb_matrix =
      skewSymmetric(c1Tc2.x(), c1Tc2.y(), c1Tc2.z()) * c1Rc2.matrix();
  EXPECT(assert_equal(expected, aEb_matrix, 1e-8));

  // Check some projections
  EXPECT(assert_equal(Point2(0, 0), pA(0), 1e-8));
  EXPECT(assert_equal(Point2(0, 0.1), pB(0), 1e-8));
  EXPECT(assert_equal(Point2(0, -1), pA(4), 1e-8));
  EXPECT(assert_equal(Point2(-1, 0.2), pB(4), 1e-8));

  // Check homogeneous version
  EXPECT(assert_equal(Vector3(-1, 0.2, 1), vB(4), 1e-8));

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA(i).transpose() * aEb_matrix * vB(i), 1e-8);

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA(i), vB(i)), 1e-7);
}

//*************************************************************************
TEST(EssentialMatrixFactor, factor) {
  Key key(1);
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor factor(key, pA(i), pB(i), model1);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Vector actual = factor.evaluateError(trueE);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values val;
    val.insert(key, trueE);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, val, 1e-5, 1e-7);
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor, ExpressionFactor) {
  Key key(1);
  for (size_t i = 0; i < 5; i++) {
    std::function<double(const EssentialMatrix &, OptionalJacobian<1, 5>)> f =
        std::bind(&EssentialMatrix::error, std::placeholders::_1, vA(i), vB(i), std::placeholders::_2);
    Expression<EssentialMatrix> E_(key);  // leaf expression
    Expression<double> expr(f, E_);       // unary expression

    // Test the derivatives using Paul's magic
    Values values;
    values.insert(key, trueE);
    EXPECT_CORRECT_EXPRESSION_JACOBIANS(expr, values, 1e-5, 1e-9);

    // Create the factor
    ExpressionFactor<double> factor(model1, 0, expr);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    vector<Matrix> Hactual(1);
    Vector actual = factor.unwhitenedError(values, Hactual);
    EXPECT(assert_equal(expected, actual, 1e-7));
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor, ExpressionFactorRotationOnly) {
  Key key(1);
  for (size_t i = 0; i < 5; i++) {
    std::function<double(const EssentialMatrix &, OptionalJacobian<1, 5>)> f =
        std::bind(&EssentialMatrix::error, std::placeholders::_1, vA(i), vB(i), std::placeholders::_2);
    std::function<EssentialMatrix(const Rot3 &, const Unit3 &,
                                    OptionalJacobian<5, 3>,
                                    OptionalJacobian<5, 2>)>
        g;
    Expression<Rot3> R_(key);
    Expression<Unit3> d_(trueDirection);
    Expression<EssentialMatrix> E_(&EssentialMatrix::FromRotationAndDirection,
                                   R_, d_);
    Expression<double> expr(f, E_);

    // Test the derivatives using Paul's magic
    Values values;
    values.insert(key, trueRotation);
    EXPECT_CORRECT_EXPRESSION_JACOBIANS(expr, values, 1e-5, 1e-9);

    // Create the factor
    ExpressionFactor<double> factor(model1, 0, expr);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    vector<Matrix> Hactual(1);
    Vector actual = factor.unwhitenedError(values, Hactual);
    EXPECT(assert_equal(expected, actual, 1e-7));
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor, minimization) {
  // Here we want to optimize directly on essential matrix constraints
  // Yi Ma's algorithm (Ma01ijcv) is a bit cumbersome to implement,
  // but GTSAM does the equivalent anyway, provided we give the right
  // factors. In this case, the factors are the constraints.

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor>(1, pA(i), pB(i), model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  initial.insert(1, initialE);
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(643.26, graph.error(initial), 1e-2);
#else
  EXPECT_DOUBLES_EQUAL(639.84, graph.error(initial), 1e-2);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(1);
  EXPECT(assert_equal(trueE, actual, 1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, actual.error(vA(i), vB(i)), 1e-6);
}

//*************************************************************************
TEST(EssentialMatrixFactor2, factor) {
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor2 factor(100, i, pA(i), pB(i), model2);

    // Check evaluation
    Point3 P1 = data.tracks[i].p, P2 = data.cameras[1].pose().transformTo(P1);
    const Point2 pi = PinholeBase::Project(P2);
    Point2 expected(pi - pB(i));

    Matrix Hactual1, Hactual2;
    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(trueE, d, Hactual1, Hactual2);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values val;
    val.insert(100, trueE);
    val.insert(i, d);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, val, 1e-5, 1e-7);
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor2, minimization) {
  // Here we want to optimize for E and inverse depths at the same time

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor2>(100, i, pA(i), pB(i), model2);

  // Check error at ground truth
  Values truth;
  truth.insert(100, trueE);
  for (size_t i = 0; i < 5; i++) {
    Point3 P1 = data.tracks[i].p;
    truth.insert(i, double(baseline / P1.z()));
  }
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Optimize
  LevenbergMarquardtParams parameters;
  // parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, truth, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(100);
  EXPECT(assert_equal(trueE, actual, 1e-1));
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(truth.at<double>(i), result.at<double>(i), 1e-1);

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
// Below we want to optimize for an essential matrix specified in a
// body coordinate frame B which is rotated with respect to the camera
// frame C, via the rotation bRc.

// The "true E" in the body frame is then
EssentialMatrix bodyE = cRb.inverse() * trueE;

//*************************************************************************
TEST(EssentialMatrixFactor3, factor) {
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor3 factor(100, i, pA(i), pB(i), cRb, model2);

    // Check evaluation
    Point3 P1 = data.tracks[i].p;
    const Point2 pi = camera2.project(P1);
    Point2 expected(pi - pB(i));

    Matrix Hactual1, Hactual2;
    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(bodyE, d, Hactual1, Hactual2);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values val;
    val.insert(100, bodyE);
    val.insert(i, d);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, val, 1e-6, 1e-7);
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor3, minimization) {
  // As before, we start with a factor graph and add constraints to it
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    // but now we specify the rotation bRc
    graph.emplace_shared<EssentialMatrixFactor3>(100, i, pA(i), pB(i), cRb,
                                                 model2);

  // Check error at ground truth
  Values truth;
  truth.insert(100, bodyE);
  for (size_t i = 0; i < 5; i++) {
    Point3 P1 = data.tracks[i].p;
    truth.insert(i, double(baseline / P1.z()));
  }
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Optimize
  LevenbergMarquardtParams parameters;
  // parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, truth, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(100);
  EXPECT(assert_equal(bodyE, actual, 1e-1));
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(truth.at<double>(i), result.at<double>(i), 1e-1);

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST(EssentialMatrixFactor4, factor) {
  Key keyE(1);
  Key keyK(2);
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor4<Cal3_S2> factor(keyE, keyK, pA(i), pB(i), model1);

    // Check evaluation
    Vector1 expected;
    expected << 0;
    Vector actual = factor.evaluateError(trueE, trueK);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values truth;
    truth.insert(keyE, trueE);
    truth.insert(keyK, trueK);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, truth, 1e-6, 1e-7);
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor4, evaluateErrorJacobiansCal3S2) {
  Key keyE(1);
  Key keyK(2);
  // initialize essential matrix
  Rot3 r = Rot3::Expmap(Vector3(M_PI / 6, M_PI / 3, M_PI / 9));
  Unit3 t(Point3(2, -1, 0.5));
  EssentialMatrix E = EssentialMatrix::FromRotationAndDirection(r, t);
  Cal3_S2 K(200, 1, 1, 10, 10);
  Values val;
  val.insert(keyE, E);
  val.insert(keyK, K);

  Point2 pA(10.0, 20.0);
  Point2 pB(12.0, 15.0);

  EssentialMatrixFactor4<Cal3_S2> f(keyE, keyK, pA, pB, model1);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-6);
}

//*************************************************************************
TEST(EssentialMatrixFactor4, evaluateErrorJacobiansCal3Bundler) {
  Key keyE(1);
  Key keyK(2);
  // initialize essential matrix
  Rot3 r = Rot3::Expmap(Vector3(0, 0, M_PI_2));
  Unit3 t(Point3(0.1, 0, 0));
  EssentialMatrix E = EssentialMatrix::FromRotationAndDirection(r, t);
  Cal3Bundler K;
  Values val;
  val.insert(keyE, E);
  val.insert(keyK, K);

  Point2 pA(-0.1, 0.5);
  Point2 pB(-0.5, -0.2);

  EssentialMatrixFactor4<Cal3Bundler> f(keyE, keyK, pA, pB, model1);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-5);
}

//*************************************************************************
TEST(EssentialMatrixFactor4, minimizationWithStrongCal3S2Prior) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3_S2>>(1, 2, pA(i), pB(i),
                                                          model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Initialization
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  initial.insert(1, initialE);
  initial.insert(2, trueK);

  // add prior factor for calibration
  Vector5 priorNoiseModelSigma;
  priorNoiseModelSigma << 10, 10, 10, 10, 10;
  graph.emplace_shared<PriorFactor<Cal3_S2>>(
      2, trueK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));

  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3_S2 actualK = result.at<Cal3_S2>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1));
  EXPECT(assert_equal(trueK, actualK, 1e-2));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(
        0,
        actualE.error(EssentialMatrix::Homogeneous(actualK.calibrate(pA(i))),
                      EssentialMatrix::Homogeneous(actualK.calibrate(pB(i)))),
        1e-6);
}

//*************************************************************************
TEST(EssentialMatrixFactor4, minimizationWithWeakCal3S2Prior) {
  // We need 7 points here as the prior on the focal length parameters is weak
  // and the initialization is noisy. So in total we are trying to optimize 7
  // DOF, with a strong prior on the remaining 3 DOF.
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 7; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3_S2>>(1, 2, pA(i), pB(i),
                                                          model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Initialization
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3_S2 initialK =
      trueK.retract((Vector(5) << 0.1, -0.1, 0.0, -0.0, 0.0).finished());
  initial.insert(1, initialE);
  initial.insert(2, initialK);

  // add prior factor for calibration
  Vector5 priorNoiseModelSigma;
  priorNoiseModelSigma << 20, 20, 1, 1, 1;
  graph.emplace_shared<PriorFactor<Cal3_S2>>(
      2, initialK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));

  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3_S2 actualK = result.at<Cal3_S2>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1));
  EXPECT(assert_equal(trueK, actualK, 1e-2));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 7; i++)
    EXPECT_DOUBLES_EQUAL(
        0,
        actualE.error(EssentialMatrix::Homogeneous(actualK.calibrate(pA(i))),
                      EssentialMatrix::Homogeneous(actualK.calibrate(pB(i)))),
        1e-5);
}

//*************************************************************************
TEST(EssentialMatrixFactor4, minimizationWithStrongCal3BundlerPrior) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3Bundler>>(1, 2, pA(i),
                                                              pB(i), model1);
  Cal3Bundler trueK(1, 0, 0, 0, 0, /*tolerance=*/5e-3);
  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3Bundler initialK = trueK;
  initial.insert(1, initialE);
  initial.insert(2, initialK);

  // add prior factor for calibration
  Vector3 priorNoiseModelSigma;
  priorNoiseModelSigma << 0.1, 0.1, 0.1;
  graph.emplace_shared<PriorFactor<Cal3Bundler>>(
      2, trueK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));

  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3Bundler actualK = result.at<Cal3Bundler>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1));
  EXPECT(assert_equal(trueK, actualK, 1e-2));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(
        0,
        actualE.error(EssentialMatrix::Homogeneous(actualK.calibrate(pA(i))),
                      EssentialMatrix::Homogeneous(actualK.calibrate(pB(i)))),
        1e-6);
}

}  // namespace example1

//*************************************************************************

namespace example2 {

const string filename = findExampleDataFile("5pointExample2.txt");
SfmData data = SfmData::FromBalFile(filename);
Rot3 aRb = data.cameras[1].pose().rotation();
Point3 aTb = data.cameras[1].pose().translation();
EssentialMatrix trueE(aRb, Unit3(aTb));

double baseline = 10;  // actual baseline of the camera

Point2 pA(size_t i) { return data.tracks[i].measurements[0].second; }
Point2 pB(size_t i) { return data.tracks[i].measurements[1].second; }

Cal3Bundler trueK = Cal3Bundler(500, 0, 0);
std::shared_ptr<Cal3Bundler> K = std::make_shared<Cal3Bundler>(trueK);
PinholeCamera<Cal3Bundler> camera2(data.cameras[1].pose(), trueK);

Vector vA(size_t i) {
  Point2 xy = trueK.calibrate(pA(i));
  return EssentialMatrix::Homogeneous(xy);
}
Vector vB(size_t i) {
  Point2 xy = trueK.calibrate(pB(i));
  return EssentialMatrix::Homogeneous(xy);
}

//*************************************************************************
TEST(EssentialMatrixFactor, extraMinimization) {
  // Additional test with camera moving in positive X direction

  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor>(1, pA(i), pB(i), model1, K);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  initial.insert(1, initialE);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(643.26, graph.error(initial), 1e-2);
#else
  EXPECT_DOUBLES_EQUAL(639.84, graph.error(initial), 1e-2);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(1);
  EXPECT(assert_equal(trueE, actual, 1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, actual.error(vA(i), vB(i)), 1e-6);
}

//*************************************************************************
TEST(EssentialMatrixFactor2, extraTest) {
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor2 factor(100, i, pA(i), pB(i), model2, K);

    // Check evaluation
    Point3 P1 = data.tracks[i].p;
    const Point2 pi = camera2.project(P1);
    Point2 expected(pi - pB(i));

    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(trueE, d);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values val;
    val.insert(100, trueE);
    val.insert(i, d);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, val, 1e-5, 1e-6);
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor2, extraMinimization) {
  // Additional test with camera moving in positive X direction

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1, assuming pixel measurements
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < data.numberTracks(); i++)
    graph.emplace_shared<EssentialMatrixFactor2>(100, i, pA(i), pB(i), model2,
                                                 K);

  // Check error at ground truth
  Values truth;
  truth.insert(100, trueE);
  for (size_t i = 0; i < data.numberTracks(); i++) {
    Point3 P1 = data.tracks[i].p;
    truth.insert(i, double(baseline / P1.z()));
  }
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Optimize
  LevenbergMarquardtParams parameters;
  // parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, truth, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(100);
  EXPECT(assert_equal(trueE, actual, 1e-1));
  for (size_t i = 0; i < data.numberTracks(); i++)
    EXPECT_DOUBLES_EQUAL(truth.at<double>(i), result.at<double>(i), 1e-1);

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST(EssentialMatrixFactor3, extraTest) {
  // The "true E" in the body frame is
  EssentialMatrix bodyE = cRb.inverse() * trueE;

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor3 factor(100, i, pA(i), pB(i), cRb, model2, K);

    // Check evaluation
    Point3 P1 = data.tracks[i].p;
    const Point2 pi = camera2.project(P1);
    Point2 expected(pi - pB(i));

    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(bodyE, d);
    EXPECT(assert_equal(expected, actual, 1e-7));

    Values val;
    val.insert(100, bodyE);
    val.insert(i, d);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, val, 1e-5, 1e-6);
  }
}

}  // namespace example2

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
