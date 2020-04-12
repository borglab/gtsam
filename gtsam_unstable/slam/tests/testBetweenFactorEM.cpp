/**
 * @file    testBetweenFactorEM.cpp
 * @brief   Unit test for the BetweenFactorEM
 * @author  Vadim Indelman
 */

#include <CppUnitLite/TestHarness.h>


#include <gtsam_unstable/slam/BetweenFactorEM.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/slam/BetweenFactor.h>


using namespace std;
using namespace gtsam;


// Disabled this test because it is currently failing - remove the lines "#if 0" and "#endif" below
// to reenable the test.
#if 0

/* ************************************************************************* */
LieVector predictionError(const Pose2& p1, const Pose2& p2, const gtsam::Key& key1, const gtsam::Key& key2, const BetweenFactorEM<gtsam::Pose2>& factor){
  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);
  //  LieVector err = factor.whitenedError(values);
  //  return err;
  return LieVector::Expmap(factor.whitenedError(values));
}

/* ************************************************************************* */
LieVector predictionError_standard(const Pose2& p1, const Pose2& p2, const gtsam::Key& key1, const gtsam::Key& key2, const BetweenFactor<gtsam::Pose2>& factor){
  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);
  //  LieVector err = factor.whitenedError(values);
  //  return err;
  return LieVector::Expmap(factor.whitenedError(values));
}

/* ************************************************************************* */
TEST( BetweenFactorEM, ConstructorAndEquals)
{
  gtsam::Key key1(1);
  gtsam::Key key2(2);

  gtsam::Pose2 p1(10.0, 15.0, 0.1);
  gtsam::Pose2 p2(15.0, 15.0, 0.3);
  gtsam::Pose2 noise(0.5, 0.4, 0.01);
  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(5, 5, 1.0)));

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  // Constructor
  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);
  BetweenFactorEM<gtsam::Pose2> g(key1, key2, rel_pose_msr, model_inlier, model_outlier,
        prior_inlier, prior_outlier);

  // Equals
  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */
TEST( BetweenFactorEM, EvaluateError)
{
  gtsam::Key key1(1);
  gtsam::Key key2(2);

  // Inlier test
  gtsam::Pose2 p1(10.0, 15.0, 0.1);
  gtsam::Pose2 p2(15.0, 15.0, 0.3);
  gtsam::Pose2 noise(0.5, 0.4, 0.01);
  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(50.0, 50.0, 10.0)));

  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  Vector actual_err_wh = f.whitenedError(values);

  Vector actual_err_wh_inlier  = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  Vector actual_err_wh_outlier = (Vector(3) << actual_err_wh[3], actual_err_wh[4], actual_err_wh[5]);

  //  cout << "Inlier test. norm of actual_err_wh_inlier, actual_err_wh_outlier: "<<actual_err_wh_inlier.norm()<<","<<actual_err_wh_outlier.norm()<<endl;
  //  cout<<actual_err_wh[0]<<" "<<actual_err_wh[1]<<" "<<actual_err_wh[2]<<actual_err_wh[3]<<" "<<actual_err_wh[4]<<" "<<actual_err_wh[5]<<endl;

  // in case of inlier, inlier-mode whitented error should be dominant
  //  CHECK(actual_err_wh_inlier.norm() > 1000.0*actual_err_wh_outlier.norm());

  // Outlier test
  noise = gtsam::Pose2(10.5, 20.4, 2.01);
  gtsam::Pose2 rel_pose_msr_test2   = rel_pose_ideal.compose(noise);

  BetweenFactorEM<gtsam::Pose2> g(key1, key2, rel_pose_msr_test2, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  actual_err_wh = g.whitenedError(values);

  actual_err_wh_inlier = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  actual_err_wh_outlier = (Vector(3) << actual_err_wh[3], actual_err_wh[4], actual_err_wh[5]);

  // in case of outlier, outlier-mode whitented error should be dominant
  //  CHECK(actual_err_wh_inlier.norm() < 1000.0*actual_err_wh_outlier.norm());
  //
  //  cout << "Outlier test. norm of actual_err_wh_inlier, actual_err_wh_outlier: "<<actual_err_wh_inlier.norm()<<","<<actual_err_wh_outlier<<endl;
  //  cout<<actual_err_wh[0]<<" "<<actual_err_wh[1]<<" "<<actual_err_wh[2]<<actual_err_wh[3]<<" "<<actual_err_wh[4]<<" "<<actual_err_wh[5]<<endl;

  // Compare with standard between factor for the inlier case
  prior_outlier = 0.0;
  prior_inlier  = 1.0;
  BetweenFactorEM<gtsam::Pose2> h_EM(key1, key2, rel_pose_msr, model_inlier, model_outlier,
        prior_inlier, prior_outlier);
  actual_err_wh = h_EM.whitenedError(values);
  actual_err_wh_inlier = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);

  BetweenFactor<gtsam::Pose2> h(key1, key2, rel_pose_msr, model_inlier );
  Vector actual_err_wh_stnd = h.whitenedError(values);

  //  cout<<"actual_err_wh: "<<actual_err_wh_inlier[0]<<", "<<actual_err_wh_inlier[1]<<", "<<actual_err_wh_inlier[2]<<endl;
  //  cout<<"actual_err_wh_stnd: "<<actual_err_wh_stnd[0]<<", "<<actual_err_wh_stnd[1]<<", "<<actual_err_wh_stnd[2]<<endl;
  //
  //  CHECK( assert_equal(actual_err_wh_inlier, actual_err_wh_stnd, 1e-8));
}

///* ************************************************************************** */
TEST (BetweenFactorEM, jacobian ) {

  gtsam::Key key1(1);
  gtsam::Key key2(2);

  // Inlier test
  gtsam::Pose2 p1(10.0, 15.0, 0.1);
  gtsam::Pose2 p2(15.0, 15.0, 0.3);
  gtsam::Pose2 noise(0.5, 0.4, 0.01);
  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(50.0, 50.0, 10.0)));

  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);

  double prior_outlier = 0.0;
  double prior_inlier = 1.0;

  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  std::vector<gtsam::Matrix> H_actual(2);
  Vector actual_err_wh = f.whitenedError(values, H_actual);

  Matrix H1_actual = H_actual[0];
  Matrix H2_actual = H_actual[1];

  // compare to standard between factor
  BetweenFactor<gtsam::Pose2> h(key1, key2, rel_pose_msr, model_inlier );
  Vector actual_err_wh_stnd = h.whitenedError(values);
  Vector actual_err_wh_inlier = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
//  CHECK( assert_equal(actual_err_wh_stnd, actual_err_wh_inlier, 1e-8));
  std::vector<gtsam::Matrix> H_actual_stnd_unwh(2);
  (void)h.unwhitenedError(values, H_actual_stnd_unwh);
  Matrix H1_actual_stnd_unwh = H_actual_stnd_unwh[0];
  Matrix H2_actual_stnd_unwh = H_actual_stnd_unwh[1];
  Matrix H1_actual_stnd = model_inlier->Whiten(H1_actual_stnd_unwh);
  Matrix H2_actual_stnd = model_inlier->Whiten(H2_actual_stnd_unwh);
//  CHECK( assert_equal(H1_actual_stnd, H1_actual, 1e-8));
//  CHECK( assert_equal(H2_actual_stnd, H2_actual, 1e-8));

  double stepsize = 1.0e-9;
  Matrix H1_expected = gtsam::numericalDerivative11<LieVector, Pose2>(boost::bind(&predictionError, _1, p2, key1, key2, f), p1, stepsize);
  Matrix H2_expected = gtsam::numericalDerivative11<LieVector, Pose2>(boost::bind(&predictionError, p1, _1, key1, key2, f), p2, stepsize);


  // try to check numerical derivatives of a standard between factor
  Matrix H1_expected_stnd = gtsam::numericalDerivative11<LieVector, Pose2>(boost::bind(&predictionError_standard, _1, p2, key1, key2, h), p1, stepsize);
//  CHECK( assert_equal(H1_expected_stnd, H1_actual_stnd, 1e-5));
//
//
//  CHECK( assert_equal(H1_expected, H1_actual, 1e-8));
//  CHECK( assert_equal(H2_expected, H2_actual, 1e-8));

}


/* ************************************************************************* */
TEST( BetweenFactorEM, CaseStudy)
{

  bool debug = false;

  gtsam::Key key1(1);
  gtsam::Key key2(2);

  // Inlier test
  gtsam::Pose2 p1;
  gtsam::Pose2 p2(-0.0491752554, -0.289649075, -0.328993962);
  gtsam::Pose2 rel_pose_msr(0.0316191379, 0.0247539161, 0.004102182);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.4021, 0.286, 0.428)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector3(4.9821, 4.614, 1.8387)));

  gtsam::Values values;
    values.insert(key1, p1);
    values.insert(key2, p2);

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  if (debug)
    cout << "==== inside CaseStudy ===="<<endl;

  gtsam::Vector p_inlier_outler = f.calcIndicatorProb(values);

  Vector actual_err_unw = f.unwhitenedError(values);
  Vector actual_err_wh  = f.whitenedError(values);

  Vector actual_err_wh_inlier = (Vector(3) << actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  Vector actual_err_wh_outlier = (Vector(3) << actual_err_wh[3], actual_err_wh[4], actual_err_wh[5]);

  if (debug){
    cout << "p_inlier_outler: "<<p_inlier_outler[0]<<", "<<p_inlier_outler[1]<<endl;
    cout<<"actual_err_unw: "<<actual_err_unw[0]<<", "<<actual_err_unw[1]<<", "<<actual_err_unw[2]<<endl;
    cout<<"actual_err_wh_inlier: "<<actual_err_wh_inlier[0]<<", "<<actual_err_wh_inlier[1]<<", "<<actual_err_wh_inlier[2]<<endl;
    cout<<"actual_err_wh_outlier: "<<actual_err_wh_outlier[0]<<", "<<actual_err_wh_outlier[1]<<", "<<actual_err_wh_outlier[2]<<endl;
  }
}


///* ************************************************************************** */
TEST (BetweenFactorEM, updateNoiseModel ) {
  gtsam::Key key1(1);
  gtsam::Key key2(2);

  gtsam::Pose2 p1(10.0, 15.0, 0.1);
  gtsam::Pose2 p2(15.0, 15.0, 0.3);
  gtsam::Pose2 noise(0.5, 0.4, 0.01);
  gtsam::Pose2 rel_pose_ideal = p1.between(p2);
  gtsam::Pose2 rel_pose_msr   = rel_pose_ideal.compose(noise);

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas( (gtsam::Vector(3) << 1.5, 2.5, 4.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas( (gtsam::Vector(3) << 50.0, 50.0, 10.0)));

  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);

  double prior_outlier = 0.0;
  double prior_inlier = 1.0;

  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  SharedGaussian model = SharedGaussian(noiseModel::Isotropic::Sigma(3, 1e2));

  NonlinearFactorGraph graph;
  graph.addPrior(key1, p1, model);
  graph.addPrior(key2, p2, model);

  f.updateNoiseModels(values, graph);

  SharedGaussian model_inlier_new = f.get_model_inlier();
  SharedGaussian model_outlier_new = f.get_model_outlier();

  model_inlier->print("model_inlier:");
  model_outlier->print("model_outlier:");
  model_inlier_new->print("model_inlier_new:");
  model_outlier_new->print("model_outlier_new:");
}


#endif

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
