/**
 * @file    testBTransformBtwRobotsUnaryFactorEM.cpp
 * @brief   Unit test for the TransformBtwRobotsUnaryFactorEM
 * @author  Vadim Indelman
 */

#include <CppUnitLite/TestHarness.h>


#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
Vector predictionError(const Pose2& org1_T_org2, const gtsam::Key& key, const TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>& factor){
  gtsam::Values values;
  values.insert(key, org1_T_org2);
  return factor.whitenedError(values);
}

/* ************************************************************************* */
//Vector predictionError_standard(const Pose2& p1, const Pose2& p2, const gtsam::Key& keyA, const gtsam::Key& keyB, const BetweenFactor<gtsam::Pose2>& factor){
//  gtsam::Values values;
//  values.insert(keyA, p1);
//  values.insert(keyB, p2);
//  //  Vector err = factor.whitenedError(values);
//  //  return err;
//  return Vector::Expmap(factor.whitenedError(values));
//}

/* ************************************************************************* */
TEST( TransformBtwRobotsUnaryFactorEM, ConstructorAndEquals)
{
  gtsam::Key key(0);
  gtsam::Key keyA(1);
  gtsam::Key keyB(2);

  gtsam::Pose2 p1(10.0, 15.0, 0.1);
  gtsam::Pose2 p2(15.0, 15.0, 0.3);
  gtsam::Pose2 noise(0.5, 0.4, 0.01);
  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(Vector3(5, 5, 1.0)));

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  gtsam::Values valA, valB;
  valA.insert(keyA, p1);
  valB.insert(keyB, p2);

  // Constructor
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g(key, rel_pose_msr, keyA, keyB, valA, valB,
      model_inlier, model_outlier,prior_inlier, prior_outlier);
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> h(key, rel_pose_msr, keyA, keyB, valA, valB,
      model_inlier, model_outlier,prior_inlier, prior_outlier);

  // Equals
  CHECK(assert_equal(g, h, 1e-5));
}

/* ************************************************************************* */
TEST( TransformBtwRobotsUnaryFactorEM, unwhitenedError)
{
  gtsam::Key key(0);
  gtsam::Key keyA(1);
  gtsam::Key keyB(2);

  gtsam::Pose2 orgA_T_1(10.0, 15.0, 0.1);
  gtsam::Pose2 orgB_T_2(15.0, 15.0, 0.3);

  gtsam::Pose2 orgA_T_orgB(100.0, 45.0, 1.8);

  gtsam::Pose2 orgA_T_2 = orgA_T_orgB.compose(orgB_T_2);

  gtsam::Pose2 rel_pose_ideal = orgA_T_1.between(orgA_T_2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal;

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(Vector3(5, 5, 1.0)));

  double prior_outlier = 0.01;
  double prior_inlier = 0.99;

  gtsam::Values valA, valB;
  valA.insert(keyA, orgA_T_1);
  valB.insert(keyB, orgB_T_2);

  // Constructor
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g(key, rel_pose_msr, keyA, keyB, valA, valB,
        model_inlier, model_outlier,prior_inlier, prior_outlier);

  gtsam::Values values;
  values.insert(key, orgA_T_orgB);
  Vector err = g.unwhitenedError(values);

  // Equals
  CHECK(assert_equal(err, Z_3x1, 1e-5));
}

/* ************************************************************************* */
TEST( TransformBtwRobotsUnaryFactorEM, unwhitenedError2)
{
  gtsam::Key key(0);
  gtsam::Key keyA(1);
  gtsam::Key keyB(2);

  gtsam::Pose2 orgA_T_currA(0.0, 0.0, 0.0);
  gtsam::Pose2 orgB_T_currB(-10.0, 15.0, 0.1);

  gtsam::Pose2 orgA_T_orgB(0.0, 0.0, 0.0);

  gtsam::Pose2 orgA_T_currB = orgA_T_orgB.compose(orgB_T_currB);

  gtsam::Pose2 rel_pose_ideal = orgA_T_currA.between(orgA_T_currB);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal;

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(Vector3(5, 5, 1.0)));

  double prior_outlier = 0.01;
  double prior_inlier = 0.99;

  gtsam::Values valA, valB;
  valA.insert(keyA, orgA_T_currA);
  valB.insert(keyB, orgB_T_currB);

  // Constructor
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g(key, rel_pose_msr, keyA, keyB, valA, valB,
        model_inlier, model_outlier,prior_inlier, prior_outlier);

  gtsam::Values values;
  values.insert(key, orgA_T_orgB);
  Vector err = g.unwhitenedError(values);

  // Equals
  CHECK(assert_equal(err, Z_3x1, 1e-5));
}

/* ************************************************************************* */
TEST( TransformBtwRobotsUnaryFactorEM, Optimize)
{
  gtsam::Key key(0);
  gtsam::Key keyA(1);
  gtsam::Key keyB(2);

  gtsam::Pose2 orgA_T_currA(0.0, 0.0, 0.0);
  gtsam::Pose2 orgB_T_currB(1.0, 2.0, 0.05);

  gtsam::Pose2 orgA_T_orgB_tr(10.0, -15.0, 0.0);
  gtsam::Pose2 orgA_T_currB_tr  = orgA_T_orgB_tr.compose(orgB_T_currB);
  gtsam::Pose2 currA_T_currB_tr = orgA_T_currA.between(orgA_T_currB_tr);

  // some error in measurements
  //  gtsam::Pose2 currA_Tmsr_currB1 = currA_T_currB_tr.compose(gtsam::Pose2(0.1, 0.02, 0.01));
  //  gtsam::Pose2 currA_Tmsr_currB2 = currA_T_currB_tr.compose(gtsam::Pose2(-0.1, 0.02, 0.01));
  //  gtsam::Pose2 currA_Tmsr_currB3 = currA_T_currB_tr.compose(gtsam::Pose2(0.1, -0.02, 0.01));
  //  gtsam::Pose2 currA_Tmsr_currB4 = currA_T_currB_tr.compose(gtsam::Pose2(0.1, 0.02, -0.01));

  // ideal measurements
  gtsam::Pose2 currA_Tmsr_currB1 = currA_T_currB_tr.compose(gtsam::Pose2(0.0, 0.0, 0.0));
  gtsam::Pose2 currA_Tmsr_currB2 = currA_Tmsr_currB1;
  gtsam::Pose2 currA_Tmsr_currB3 = currA_Tmsr_currB1;
  gtsam::Pose2 currA_Tmsr_currB4 = currA_Tmsr_currB1;

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(Vector3(5, 5, 1.0)));

  double prior_outlier = 0.01;
  double prior_inlier = 0.99;

  gtsam::Values valA, valB;
  valA.insert(keyA, orgA_T_currA);
  valB.insert(keyB, orgB_T_currB);

  // Constructor
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g1(key, currA_Tmsr_currB1, keyA, keyB, valA, valB,
        model_inlier, model_outlier,prior_inlier, prior_outlier);

  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g2(key, currA_Tmsr_currB2, keyA, keyB, valA, valB,
          model_inlier, model_outlier,prior_inlier, prior_outlier);

  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g3(key, currA_Tmsr_currB3, keyA, keyB, valA, valB,
          model_inlier, model_outlier,prior_inlier, prior_outlier);

  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g4(key, currA_Tmsr_currB4, keyA, keyB, valA, valB,
          model_inlier, model_outlier,prior_inlier, prior_outlier);

  gtsam::Values values;
  values.insert(key, gtsam::Pose2());

  gtsam::NonlinearFactorGraph graph;
  graph.push_back(g1);
  graph.push_back(g2);
  graph.push_back(g3);
  graph.push_back(g4);

  gtsam::GaussNewtonParams params;
  gtsam::GaussNewtonOptimizer optimizer(graph, values, params);
  gtsam::Values result = optimizer.optimize();

  gtsam::Pose2 orgA_T_orgB_opt = result.at<gtsam::Pose2>(key);

  CHECK(assert_equal(orgA_T_orgB_opt, orgA_T_orgB_tr, 1e-5));
}


/* ************************************************************************* */
TEST( TransformBtwRobotsUnaryFactorEM, Jacobian)
{
  gtsam::Key key(0);
  gtsam::Key keyA(1);
  gtsam::Key keyB(2);

  gtsam::Pose2 orgA_T_1(10.0, 15.0, 0.1);
  gtsam::Pose2 orgB_T_2(15.0, 15.0, 0.3);

  gtsam::Pose2 orgA_T_orgB(100.0, 45.0, 1.8);

  gtsam::Pose2 orgA_T_2 = orgA_T_orgB.compose(orgB_T_2);

  gtsam::Pose2 noise(0.5, 0.4, 0.01);

  gtsam::Pose2 rel_pose_ideal = orgA_T_1.between(orgA_T_2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(Vector3(5, 5, 1.0)));

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  gtsam::Values valA, valB;
  valA.insert(keyA, orgA_T_1);
  valB.insert(keyB, orgB_T_2);

  // Constructor
  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> g(key, rel_pose_msr, keyA, keyB, valA, valB,
        model_inlier, model_outlier,prior_inlier, prior_outlier);

  gtsam::Values values;
  values.insert(key, orgA_T_orgB);

  std::vector<gtsam::Matrix> H_actual(1);
  Vector actual_err_wh = g.whitenedError(values, H_actual);

  Matrix H1_actual = H_actual[0];

  double stepsize = 1.0e-9;
  Matrix H1_expected = gtsam::numericalDerivative11<Vector, Pose2>(
      std::bind(&predictionError, std::placeholders::_1, key, g), orgA_T_orgB,
      stepsize);
  //  CHECK( assert_equal(H1_expected, H1_actual, 1e-5));
}
/////* ************************************************************************** */
//TEST (TransformBtwRobotsUnaryFactorEM, jacobian ) {
//
//  gtsam::Key keyA(1);
//  gtsam::Key keyB(2);
//
//  // Inlier test
//  gtsam::Pose2 p1(10.0, 15.0, 0.1);
//  gtsam::Pose2 p2(15.0, 15.0, 0.3);
//  gtsam::Pose2 noise(0.5, 0.4, 0.01);
//  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
//  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);
//
//  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.05)));
//  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(50.0, 50.0, 10.0)));
//
//  gtsam::Values values;
//  values.insert(keyA, p1);
//  values.insert(keyB, p2);
//
//  double prior_outlier = 0.0;
//  double prior_inlier = 1.0;
//
//  TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> f(keyA, keyB, rel_pose_msr, model_inlier, model_outlier,
//      prior_inlier, prior_outlier);
//
//  std::vector<gtsam::Matrix> H_actual(2);
//  Vector actual_err_wh = f.whitenedError(values, H_actual);
//
//  Matrix H1_actual = H_actual[0];
//  Matrix H2_actual = H_actual[1];
//
//  // compare to standard between factor
//  BetweenFactor<gtsam::Pose2> h(keyA, keyB, rel_pose_msr, model_inlier );
//  Vector actual_err_wh_stnd = h.whitenedError(values);
//  Vector actual_err_wh_inlier = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
//  CHECK( assert_equal(actual_err_wh_stnd, actual_err_wh_inlier, 1e-8));
//  std::vector<gtsam::Matrix> H_actual_stnd_unwh(2);
//  (void)h.unwhitenedError(values, H_actual_stnd_unwh);
//  Matrix H1_actual_stnd_unwh = H_actual_stnd_unwh[0];
//  Matrix H2_actual_stnd_unwh = H_actual_stnd_unwh[1];
//  Matrix H1_actual_stnd = model_inlier->Whiten(H1_actual_stnd_unwh);
//  Matrix H2_actual_stnd = model_inlier->Whiten(H2_actual_stnd_unwh);
////  CHECK( assert_equal(H1_actual_stnd, H1_actual, 1e-8));
////  CHECK( assert_equal(H2_actual_stnd, H2_actual, 1e-8));
//
//  double stepsize = 1.0e-9;
//  Matrix H1_expected = gtsam::numericalDerivative11<Vector, Pose2>(std::bind(&predictionError, std::placeholders::_1, p2, keyA, keyB, f), p1, stepsize);
//  Matrix H2_expected = gtsam::numericalDerivative11<Vector, Pose2>(std::bind(&predictionError, p1, std::placeholders::_1, keyA, keyB, f), p2, stepsize);
//
//
//  // try to check numerical derivatives of a standard between factor
//  Matrix H1_expected_stnd = gtsam::numericalDerivative11<Vector, Pose2>(std::bind(&predictionError_standard, std::placeholders::_1, p2, keyA, keyB, h), p1, stepsize);
//  CHECK( assert_equal(H1_expected_stnd, H1_actual_stnd, 1e-5));
//
//
//  CHECK( assert_equal(H1_expected, H1_actual, 1e-8));
//  CHECK( assert_equal(H2_expected, H2_actual, 1e-8));
//
//}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
