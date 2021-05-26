/*
 * @file testEssentialMatrixFactor.cpp
 * @brief Test EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/slam/EssentialMatrixFactor.h>

#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/Matrix.h>
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
// TODO: maybe default value not good; assert with 0th
Cal3_S2 trueK = Cal3_S2();
PinholeCamera<Cal3_S2> camera2(data.cameras[1].pose(), trueK);
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
Vector vA(size_t i) {
  return EssentialMatrix::Homogeneous(pA(i));
}
Vector vB(size_t i) {
  return EssentialMatrix::Homogeneous(pB(i));
}

const string filename_18p = findExampleDataFile("18points.txt");
SfmData data_18p;
bool readOK_18p = readBAL(filename_18p, data_18p);
Rot3 c1Rc2_18p = data_18p.cameras[0].pose().rotation().between(data_18p.cameras[1].pose().rotation());
Point3 c1Tc2_18p = data_18p.cameras[0].pose().rotation().unrotate(data_18p.cameras[1].pose().translation() - data_18p.cameras[0].pose().translation());
Rot3 trueRotation_18p(c1Rc2_18p);
Unit3 trueDirection_18p(c1Tc2_18p);
EssentialMatrix trueE_18p(trueRotation_18p, trueDirection_18p);

Point2 pA_18p(size_t i) {
  return data_18p.tracks[i].measurements[0].second;
}
Point2 pB_18p(size_t i) {
  return data_18p.tracks[i].measurements[1].second;
}
Vector vA_18p(size_t i) {
  return EssentialMatrix::Homogeneous(pA_18p(i));
}
Vector vB_18p(size_t i) {
  return EssentialMatrix::Homogeneous(pB_18p(i));
}
//*************************************************************************
TEST (EssentialMatrixFactor, testData) {
  CHECK(readOK);
  CHECK(readOK_18p);

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
  EXPECT(assert_equal(Vector3(-1, 0.2, 1), vB(4), 1e-8));

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA(i).transpose() * aEb_matrix * vB(i), 1e-8);

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA(i), vB(i)), 1e-7);
}

//*************************************************************************
TEST (EssentialMatrixFactor, factor) {
  Key key(1);
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor factor(key, pA(i), pB(i), model1);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix Hactual;
    Vector actual = factor.evaluateError(trueE, Hactual);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected;
    typedef Eigen::Matrix<double,1,1> Vector1;
    Hexpected = numericalDerivative11<Vector1, EssentialMatrix>(
        boost::bind(&EssentialMatrixFactor::evaluateError, &factor, _1,
            boost::none), trueE);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected, Hactual, 1e-8));
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor, ExpressionFactor) {
  Key key(1);
  for (size_t i = 0; i < 5; i++) {
    boost::function<double(const EssentialMatrix&, OptionalJacobian<1, 5>)> f =
        boost::bind(&EssentialMatrix::error, _1, vA(i), vB(i), _2);
    Expression<EssentialMatrix> E_(key); // leaf expression
    Expression<double> expr(f, E_); // unary expression

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
    boost::function<double(const EssentialMatrix&, OptionalJacobian<1, 5>)> f =
        boost::bind(&EssentialMatrix::error, _1, vA(i), vB(i), _2);
    boost::function<EssentialMatrix(const Rot3&, const Unit3&, OptionalJacobian<5, 3>,
                                    OptionalJacobian<5, 2>)> g;
    Expression<Rot3> R_(key);
    Expression<Unit3> d_(trueDirection);
    Expression<EssentialMatrix> E_(&EssentialMatrix::FromRotationAndDirection, R_, d_);
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
TEST (EssentialMatrixFactor, minimization) {
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
  EssentialMatrix initialE = trueE.retract(
      (Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
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
TEST (EssentialMatrixFactor2, factor) {
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

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected1, Hexpected2;
    boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
        &EssentialMatrixFactor2::evaluateError, &factor, _1, _2, boost::none,
        boost::none);
    Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, trueE, d);
    Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, trueE, d);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected1, Hactual1, 1e-8));
    EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
  }
}

//*************************************************************************
TEST (EssentialMatrixFactor2, minimization) {
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
TEST (EssentialMatrixFactor3, factor) {

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

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected1, Hexpected2;
    boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
        &EssentialMatrixFactor3::evaluateError, &factor, _1, _2, boost::none,
        boost::none);
    Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, bodyE, d);
    Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, bodyE, d);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected1, Hactual1, 1e-8));
    EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
  }
}

//*************************************************************************
TEST (EssentialMatrixFactor3, minimization) {

  // As before, we start with a factor graph and add constraints to it
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    // but now we specify the rotation bRc
    graph.emplace_shared<EssentialMatrixFactor3>(100, i, pA(i), pB(i), cRb, model2);

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
    Matrix HEactual;
    Matrix HKactual;
    Vector actual = factor.evaluateError(trueE, trueK, HEactual, HKactual);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix HEexpected;
    Matrix HKexpected;
    typedef Eigen::Matrix<double, 1, 1> Vector1;
    boost::function<Vector(const EssentialMatrix &, const Cal3_S2 &)> f =
        boost::bind(&EssentialMatrixFactor4<Cal3_S2>::evaluateError, factor, _1,
                    _2, boost::none, boost::none);
    HEexpected = numericalDerivative21<Vector1, EssentialMatrix, Cal3_S2>(
        f, trueE, trueK);
    HKexpected = numericalDerivative22<Vector1, EssentialMatrix, Cal3_S2>(
        f, trueE, trueK);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(HEexpected, HEactual, 1e-8));
    EXPECT(assert_equal(HKexpected, HKactual, 1e-8));
  }
}

//*************************************************************************
TEST(EssentialMatrixFactor4, evaluateErrorJacobians) {
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
TEST(EssentialMatrixFactor4, evaluateErrorJacobiansBundler) {
  Key keyE(1);
  Key keyK(2);
  // initialize essential matrix
  Rot3 r = Rot3::Expmap(Vector3(0, 0, M_PI/6));
  Unit3 t(Point3(0.1, 0, 0));
  EssentialMatrix E = EssentialMatrix::FromRotationAndDirection(r, t);
  Cal3Bundler K;
  Values val;
  val.insert(keyE, E);
  val.insert(keyK, K);
  Point2 pA(-0.1, 0.5);
  Point2 pB(-0.5, -0.2);

  EssentialMatrixFactor4<Cal3Bundler> f(keyE, keyK, pA, pB, model1);
  std::cout << "factor error " << f.evaluateError(E, K);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, val, 1e-5, 1e-5);
}



void PrintHessianErrors(const NonlinearFactorGraph& graph, const Values& gt) {
  boost::shared_ptr<GaussianFactorGraph> gaussian_graph = graph.linearize(gt);
  std::pair<Matrix, Vector> jacobian_error = gaussian_graph->jacobian();
  std::cout << "jacobian " << std::endl << jacobian_error.first << std::endl;
  std::pair<Matrix, Vector> hessian = gaussian_graph->hessian();
  std::cout << "hessian " << std::endl << hessian.first << std::endl;
  Matrix U, V;
  Vector S;
  svd(hessian.first, U, S, V);
  std::cout << " U " << std::endl << U << std::endl;
  std::cout << " S " << std::endl << S << std::endl;
  std::cout << " V " << std::endl << V << std::endl;
}

//*************************************************************************
TEST(EssentialMatrixFactor4, minimization) {
  // As before, we start with a factor graph and add constraints to it
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3_S2>>(1, 2, pA(i), pB(i),
                                                          model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  initial.insert(1, initialE);
  initial.insert(2, trueK); 

  // add prior factor for calibration
  Vector5 priorNoiseModelSigma;
  priorNoiseModelSigma << 10, 10, 10, 10, 10;
  graph.emplace_shared<PriorFactor<Cal3_S2>>(2, trueK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));
  
  // PrintHessianErrors(graph, truth);
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3_S2 actualK = result.at<Cal3_S2>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1)); // TODO: fix the tolerance
  EXPECT(assert_equal(trueK, actualK, 1e-2)); // TODO: fix the tolerance

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

TEST(EssentialMatrixFactor4, minimization_7point) {
  // As before, we start with a factor graph and add constraints to it
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 7; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3_S2>>(1, 2, pA_18p(i), pB_18p(i),
                                                          model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE_18p);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE_18p.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3_S2 initialK =
      trueK.retract((Vector(5) << 0.1, -0.1, 0.0, -0.0, 0.0).finished());
  initialE.print("Initial E");
  initialK.print("Initial K");
  initial.insert(1, initialE);
  initial.insert(2, initialK); 

  // add prior factor for calibration
  Vector5 priorNoiseModelSigma;
  priorNoiseModelSigma << 100, 100, 10, 10, 10;
  graph.emplace_shared<PriorFactor<Cal3_S2>>(2, initialK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));
  
  // PrintHessianErrors(graph, truth);
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3_S2 actualK = result.at<Cal3_S2>(2);
  EXPECT(assert_equal(trueE_18p, actualE, 1e-1)); // TODO: fix the tolerance
  EXPECT(assert_equal(trueK, actualK, 1e-2)); // TODO: fix the tolerance

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 7; i++)
    EXPECT_DOUBLES_EQUAL(
        0,
        actualE.error(EssentialMatrix::Homogeneous(actualK.calibrate(pA_18p(i))),
                      EssentialMatrix::Homogeneous(actualK.calibrate(pB_18p(i)))),
        1e-5);
}

TEST(EssentialMatrixFactor4, minimization_7point_cal3bundler) {
  // As before, we start with a factor graph and add constraints to it
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 9; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3Bundler>>(1, 2, pA_18p(i), pB_18p(i),
                                                          model1);
  Cal3Bundler trueK_bundler;
  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE_18p);
  truth.insert(2, trueK_bundler);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE_18p.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3Bundler initialK =
      trueK_bundler.retract((Vector(3) << 1, 0.0, 0.0).finished());
  initialE.print("Initial E");
  initialK.print("Initial K");
  initial.insert(1, initialE);
  initial.insert(2, initialK); 

  // add prior factor for calibration
  Vector3 priorNoiseModelSigma;
  priorNoiseModelSigma << 1e3, 1e2, 1e2;
  graph.emplace_shared<PriorFactor<Cal3Bundler>>(2, initialK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));
  
  // PrintHessianErrors(graph, truth);
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3Bundler actualK = result.at<Cal3Bundler>(2);
  EXPECT(assert_equal(trueE_18p, actualE, 1e-1)); // TODO: fix the tolerance
  EXPECT(assert_equal(trueK_bundler, actualK, 1e-2)); // TODO: fix the tolerance

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 7; i++)
    EXPECT_DOUBLES_EQUAL(
        0,
        actualE.error(EssentialMatrix::Homogeneous(actualK.calibrate(pA_18p(i))),
                      EssentialMatrix::Homogeneous(actualK.calibrate(pB_18p(i)))),
        1e-6);
}


}  // namespace example1

//*************************************************************************

namespace example2 {

const string filename = findExampleDataFile("5pointExample2.txt");
SfmData data;
bool readOK = readBAL(filename, data);
Rot3 aRb = data.cameras[1].pose().rotation();
Point3 aTb = data.cameras[1].pose().translation();
EssentialMatrix trueE(aRb, Unit3(aTb));

double baseline = 10; // actual baseline of the camera

Point2 pA(size_t i) {
  return data.tracks[i].measurements[0].second;
}
Point2 pB(size_t i) {
  return data.tracks[i].measurements[1].second;
}

Cal3Bundler trueK = Cal3Bundler(500, 0, 0);
boost::shared_ptr<Cal3Bundler> K = boost::make_shared<Cal3Bundler>(trueK);
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
  EssentialMatrix initialE = trueE.retract(
      (Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
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
TEST (EssentialMatrixFactor2, extraTest) {
  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor2 factor(100, i, pA(i), pB(i), model2, K);

    // Check evaluation
    Point3 P1 = data.tracks[i].p;
    const Point2 pi = camera2.project(P1);
    Point2 expected(pi - pB(i));

    Matrix Hactual1, Hactual2;
    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(trueE, d, Hactual1, Hactual2);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected1, Hexpected2;
    boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
        &EssentialMatrixFactor2::evaluateError, &factor, _1, _2, boost::none,
        boost::none);
    Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, trueE, d);
    Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, trueE, d);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected1, Hactual1, 1e-6));
    EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
  }
}

//*************************************************************************
TEST (EssentialMatrixFactor2, extraMinimization) {
  // Additional test with camera moving in positive X direction

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1, assuming pixel measurements
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < data.number_tracks(); i++)
    graph.emplace_shared<EssentialMatrixFactor2>(100, i, pA(i), pB(i), model2, K);

  // Check error at ground truth
  Values truth;
  truth.insert(100, trueE);
  for (size_t i = 0; i < data.number_tracks(); i++) {
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
  for (size_t i = 0; i < data.number_tracks(); i++)
    EXPECT_DOUBLES_EQUAL(truth.at<double>(i), result.at<double>(i), 1e-1);

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST (EssentialMatrixFactor3, extraTest) {

  // The "true E" in the body frame is
  EssentialMatrix bodyE = cRb.inverse() * trueE;

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor3 factor(100, i, pA(i), pB(i), cRb, model2, K);

    // Check evaluation
    Point3 P1 = data.tracks[i].p;
    const Point2 pi = camera2.project(P1);
    Point2 expected(pi - pB(i));

    Matrix Hactual1, Hactual2;
    double d(baseline / P1.z());
    Vector actual = factor.evaluateError(bodyE, d, Hactual1, Hactual2);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected1, Hexpected2;
    boost::function<Vector(const EssentialMatrix&, double)> f = boost::bind(
        &EssentialMatrixFactor3::evaluateError, &factor, _1, _2, boost::none,
        boost::none);
    Hexpected1 = numericalDerivative21<Vector2, EssentialMatrix, double>(f, bodyE, d);
    Hexpected2 = numericalDerivative22<Vector2, EssentialMatrix, double>(f, bodyE, d);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected1, Hactual1, 1e-6));
    EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
  }
}
/*
TEST(EssentialMatrixFactor4, extraMinimization) {
  // Additional test with camera moving in positive X direction

  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 5; i++)
    graph.emplace_shared<EssentialMatrixFactor4<Cal3Bundler>>(1, 2, pA(i),
                                                              pB(i), model1);

  // Check error at ground truth
  Values truth;
  truth.insert(1, trueE);
  truth.insert(2, trueK);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE =
      trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished());
  Cal3Bundler initialK =
      trueK.retract((Vector(3) << 0.1, -0.01, 0.01).finished());
  initial.insert(1, initialE);
  initial.insert(2, initialK);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(643.62, graph.error(initial), 1e-2);
#else
  EXPECT_DOUBLES_EQUAL(639.84, graph.error(initial), 1e-2); // TODO: fix this
#endif

  // add prior factor on calibration
  Vector3 priorNoiseModelSigma;
  priorNoiseModelSigma << 0.3, 0.03, 0.03;
  graph.emplace_shared<PriorFactor<Cal3Bundler>>(2, initialK, noiseModel::Diagonal::Sigmas(priorNoiseModelSigma));

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actualE = result.at<EssentialMatrix>(1);
  Cal3Bundler actualK = result.at<Cal3Bundler>(2);
  EXPECT(assert_equal(trueE, actualE, 1e-1)); // TODO: tighten tolerance
  EXPECT(assert_equal(trueK, actualK, 1e-1)); // TODO: tighten tolerance

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
*/
}  // namespace example2

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

