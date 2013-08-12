/**
 * @file    testBetweenFactorEM.cpp
 * @brief   Unit test for the BetweenFactorEM
 * @author  Vadim Indelman
 */

#include <CppUnitLite/TestHarness.h>


#include <gtsam_unstable/slam/BetweenFactorEM.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/slam/BetweenFactor.h>

//#include <gtsam/nonlinear/NonlinearOptimizer.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/linear/GaussianSequentialSolver.h>


using namespace std;
using namespace gtsam;


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

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 5, 5, 1.0)));

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

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 50.0, 50.0, 10.0)));

  gtsam::Values values;
  values.insert(key1, p1);
  values.insert(key2, p2);

  double prior_outlier = 0.5;
  double prior_inlier = 0.5;

  BetweenFactorEM<gtsam::Pose2> f(key1, key2, rel_pose_msr, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  Vector actual_err_wh = f.whitenedError(values);

  Vector actual_err_wh_inlier = Vector_(3, actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  Vector actual_err_wh_outlier = Vector_(3, actual_err_wh[3], actual_err_wh[4], actual_err_wh[5]);

  // in case of inlier, inlier-mode whitented error should be dominant
  CHECK(actual_err_wh_inlier.norm() > 1000.0*actual_err_wh_outlier.norm());

  cout << "Inlier test. norm of actual_err_wh_inlier, actual_err_wh_outlier: "<<actual_err_wh_inlier.norm()<<","<<actual_err_wh_outlier.norm()<<endl;
  cout<<actual_err_wh[0]<<" "<<actual_err_wh[1]<<" "<<actual_err_wh[2]<<actual_err_wh[3]<<" "<<actual_err_wh[4]<<" "<<actual_err_wh[5]<<endl;


  // Outlier test
  noise = gtsam::Pose2(10.5, 20.4, 2.01);
  gtsam::Pose2 rel_pose_msr_test2   = rel_pose_ideal.compose(noise);

  BetweenFactorEM<gtsam::Pose2> g(key1, key2, rel_pose_msr_test2, model_inlier, model_outlier,
      prior_inlier, prior_outlier);

  actual_err_wh = g.whitenedError(values);

  actual_err_wh_inlier = Vector_(3, actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  actual_err_wh_outlier = Vector_(3, actual_err_wh[3], actual_err_wh[4], actual_err_wh[5]);

  // in case of outlier, outlier-mode whitented error should be dominant
  CHECK(actual_err_wh_inlier.norm() < 1000.0*actual_err_wh_outlier.norm());

  cout << "Outlier test. norm of actual_err_wh_inlier, actual_err_wh_outlier: "<<actual_err_wh_inlier.norm()<<","<<actual_err_wh_outlier<<endl;
  cout<<actual_err_wh[0]<<" "<<actual_err_wh[1]<<" "<<actual_err_wh[2]<<actual_err_wh[3]<<" "<<actual_err_wh[4]<<" "<<actual_err_wh[5]<<endl;

  // Compare with standard between factor for the inlier case
  prior_outlier = 0.0;
  prior_inlier  = 1.0;
  BetweenFactorEM<gtsam::Pose2> h_EM(key1, key2, rel_pose_msr, model_inlier, model_outlier,
        prior_inlier, prior_outlier);
  actual_err_wh = h_EM.whitenedError(values);
  actual_err_wh_inlier = Vector_(3, actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);

  BetweenFactor<gtsam::Pose2> h(key1, key2, rel_pose_msr, model_inlier );
  Vector actual_err_wh_stnd = h.whitenedError(values);

  cout<<"actual_err_wh: "<<actual_err_wh_inlier[0]<<", "<<actual_err_wh_inlier[1]<<", "<<actual_err_wh_inlier[2]<<endl;
  cout<<"actual_err_wh_stnd: "<<actual_err_wh_stnd[0]<<", "<<actual_err_wh_stnd[1]<<", "<<actual_err_wh_stnd[2]<<endl;

  CHECK( assert_equal(actual_err_wh_inlier, actual_err_wh_stnd, 1e-8));
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

  SharedGaussian model_inlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 0.5, 0.5, 0.05)));
  SharedGaussian model_outlier(noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 50.0, 50.0, 10.0)));

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
  Vector actual_err_wh_inlier = Vector_(3, actual_err_wh[0], actual_err_wh[1], actual_err_wh[2]);
  CHECK( assert_equal(actual_err_wh_stnd, actual_err_wh_inlier, 1e-8));
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
  CHECK( assert_equal(H1_expected_stnd, H1_actual_stnd, 1e-5));


  CHECK( assert_equal(H1_expected, H1_actual, 1e-8));
  CHECK( assert_equal(H2_expected, H2_actual, 1e-8));

}

/* ************************************************************************* */
TEST( InertialNavFactor, Equals)
{
//  gtsam::Key Pose1(11);
//  gtsam::Key Pose2(12);
//  gtsam::Key Vel1(21);
//  gtsam::Key Vel2(22);
//  gtsam::Key Bias1(31);
//
//  Vector measurement_acc(Vector_(3,0.1,0.2,0.4));
//  Vector measurement_gyro(Vector_(3, -0.2, 0.5, 0.03));
//
//  double measurement_dt(0.1);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> f(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> g(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor, Predict)
{
//  gtsam::Key PoseKey1(11);
//  gtsam::Key PoseKey2(12);
//  gtsam::Key VelKey1(21);
//  gtsam::Key VelKey2(22);
//  gtsam::Key BiasKey1(31);
//
//  double measurement_dt(0.1);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//
//  // First test: zero angular motion, some acceleration
//  Vector measurement_acc(Vector_(3,0.1,0.2,0.3-9.81));
//  Vector measurement_gyro(Vector_(3, 0.0, 0.0, 0.0));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//
//  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
//  LieVector Vel1(3, 0.50, -0.50, 0.40);
//  imuBias::ConstantBias Bias1;
//  Pose3 expectedPose2(Rot3(), Point3(2.05, 0.95, 3.04));
//  LieVector expectedVel2(3, 0.51, -0.48, 0.43);
//  Pose3 actualPose2;
//  LieVector actualVel2;
//  f.predict(Pose1, Vel1, Bias1, actualPose2, actualVel2);
//
//  CHECK(assert_equal(expectedPose2, actualPose2, 1e-5));
//  CHECK(assert_equal(expectedVel2, actualVel2, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor, ErrorPosVel)
{
//  gtsam::Key PoseKey1(11);
//  gtsam::Key PoseKey2(12);
//  gtsam::Key VelKey1(21);
//  gtsam::Key VelKey2(22);
//  gtsam::Key BiasKey1(31);
//
//  double measurement_dt(0.1);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//
//  // First test: zero angular motion, some acceleration
//  Vector measurement_acc(Vector_(3,0.1,0.2,0.3-9.81));
//  Vector measurement_gyro(Vector_(3, 0.0, 0.0, 0.0));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//
//  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
//  Pose3 Pose2(Rot3(), Point3(2.05, 0.95, 3.04));
//  LieVector Vel1(3, 0.50, -0.50, 0.40);
//  LieVector Vel2(3, 0.51, -0.48, 0.43);
//  imuBias::ConstantBias Bias1;
//
//  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
//  Vector ExpectedErr(zero(9));
//
//  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor, ErrorRot)
{
//  gtsam::Key PoseKey1(11);
//  gtsam::Key PoseKey2(12);
//  gtsam::Key VelKey1(21);
//  gtsam::Key VelKey2(22);
//  gtsam::Key BiasKey1(31);
//
//  double measurement_dt(0.1);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//  // Second test: zero angular motion, some acceleration
//  Vector measurement_acc(Vector_(3,0.0,0.0,0.0-9.81));
//  Vector measurement_gyro(Vector_(3, 0.1, 0.2, 0.3));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//
//  Pose3 Pose1(Rot3(), Point3(2.0,1.0,3.0));
//  Pose3 Pose2(Rot3::Expmap(measurement_gyro*measurement_dt), Point3(2.0,1.0,3.0));
//  LieVector Vel1(3,0.0,0.0,0.0);
//  LieVector Vel2(3,0.0,0.0,0.0);
//  imuBias::ConstantBias Bias1;
//
//  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
//  Vector ExpectedErr(zero(9));
//
//  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor, ErrorRotPosVel)
{
//  gtsam::Key PoseKey1(11);
//  gtsam::Key PoseKey2(12);
//  gtsam::Key VelKey1(21);
//  gtsam::Key VelKey2(22);
//  gtsam::Key BiasKey1(31);
//
//  double measurement_dt(0.1);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//  // Second test: zero angular motion, some acceleration - generated in matlab
//  Vector measurement_acc(Vector_(3, 6.501390843381716,  -6.763926150509185,  -2.300389940090343));
//  Vector measurement_gyro(Vector_(3, 0.1, 0.2, 0.3));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//
//  Rot3 R1(0.487316618,   0.125253866,   0.86419557,
//       0.580273724,  0.693095498, -0.427669306,
//      -0.652537293,  0.709880342,  0.265075427);
//  Point3 t1(2.0,1.0,3.0);
//  Pose3 Pose1(R1, t1);
//  LieVector Vel1(3,0.5,-0.5,0.4);
//  Rot3 R2(0.473618898,   0.119523052,  0.872582019,
//       0.609241153,   0.67099888, -0.422594037,
//      -0.636011287,  0.731761397,  0.244979388);
//  Point3 t2(2.052670960415706,   0.977252139079380,   2.942482135362800);
//  Pose3 Pose2(R2, t2);
//  LieVector Vel2(3,0.510000000000000,  -0.480000000000000,   0.430000000000000);
//  imuBias::ConstantBias Bias1;
//
//  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
//  Vector ExpectedErr(zero(9));
//
//  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}


/* ************************************************************************* */
TEST (InertialNavFactor, Jacobian ) {

//  gtsam::Key PoseKey1(11);
//  gtsam::Key PoseKey2(12);
//  gtsam::Key VelKey1(21);
//  gtsam::Key VelKey2(22);
//  gtsam::Key BiasKey1(31);
//
//  double measurement_dt(0.01);
//  Vector world_g(Vector_(3, 0.0, 0.0, 9.81));
//  Vector world_rho(Vector_(3, 0.0, -1.5724e-05, 0.0)); // NED system
//  gtsam::Vector ECEF_omega_earth(Vector_(3, 0.0, 0.0, 7.292115e-5));
//  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);
//
//  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));
//
//  Vector measurement_acc(Vector_(3, 6.501390843381716,  -6.763926150509185,  -2.300389940090343));
//  Vector measurement_gyro(Vector_(3, 3.14, 3.14/2, -3.14));
//
//  InertialNavFactor<Pose3, LieVector, imuBias::ConstantBias> factor(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
//
//  Rot3 R1(0.487316618,   0.125253866,   0.86419557,
//       0.580273724,  0.693095498, -0.427669306,
//      -0.652537293,  0.709880342,  0.265075427);
//  Point3 t1(2.0,1.0,3.0);
//  Pose3 Pose1(R1, t1);
//  LieVector Vel1(3,0.5,-0.5,0.4);
//  Rot3 R2(0.473618898,   0.119523052,  0.872582019,
//       0.609241153,   0.67099888, -0.422594037,
//      -0.636011287,  0.731761397,  0.244979388);
//  Point3 t2(2.052670960415706,   0.977252139079380,   2.942482135362800);
//  Pose3 Pose2(R2, t2);
//  LieVector Vel2(3,0.510000000000000,  -0.480000000000000,   0.430000000000000);
//  imuBias::ConstantBias Bias1;
//
//  Matrix H1_actual, H2_actual, H3_actual, H4_actual, H5_actual;
//
//  Vector ActualErr(factor.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2, H1_actual, H2_actual, H3_actual, H4_actual, H5_actual));
//
//  // Checking for Pose part in the jacobians
//  // ******
//  Matrix H1_actualPose(H1_actual.block(0,0,6,H1_actual.cols()));
//  Matrix H2_actualPose(H2_actual.block(0,0,6,H2_actual.cols()));
//  Matrix H3_actualPose(H3_actual.block(0,0,6,H3_actual.cols()));
//  Matrix H4_actualPose(H4_actual.block(0,0,6,H4_actual.cols()));
//  Matrix H5_actualPose(H5_actual.block(0,0,6,H5_actual.cols()));
//
//  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
//  gtsam::Matrix H1_expectedPose, H2_expectedPose, H3_expectedPose, H4_expectedPose, H5_expectedPose;
//  H1_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
//  H2_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
//  H3_expectedPose = gtsam::numericalDerivative11<Pose3, imuBias::ConstantBias>(boost::bind(&predictionErrorPose, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
//  H4_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
//  H5_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);
//
//  // Verify they are equal for this choice of state
//  CHECK( gtsam::assert_equal(H1_expectedPose, H1_actualPose, 1e-6));
//  CHECK( gtsam::assert_equal(H2_expectedPose, H2_actualPose, 1e-6));
//  CHECK( gtsam::assert_equal(H3_expectedPose, H3_actualPose, 1e-6));
//  CHECK( gtsam::assert_equal(H4_expectedPose, H4_actualPose, 1e-6));
//  CHECK( gtsam::assert_equal(H5_expectedPose, H5_actualPose, 1e-6));
//
//  // Checking for Vel part in the jacobians
//  // ******
//  Matrix H1_actualVel(H1_actual.block(6,0,3,H1_actual.cols()));
//  Matrix H2_actualVel(H2_actual.block(6,0,3,H2_actual.cols()));
//  Matrix H3_actualVel(H3_actual.block(6,0,3,H3_actual.cols()));
//  Matrix H4_actualVel(H4_actual.block(6,0,3,H4_actual.cols()));
//  Matrix H5_actualVel(H5_actual.block(6,0,3,H5_actual.cols()));
//
//  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
//  gtsam::Matrix H1_expectedVel, H2_expectedVel, H3_expectedVel, H4_expectedVel, H5_expectedVel;
//  H1_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
//  H2_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
//  H3_expectedVel = gtsam::numericalDerivative11<LieVector, imuBias::ConstantBias>(boost::bind(&predictionErrorVel, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
//  H4_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
//  H5_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);
//
//  // Verify they are equal for this choice of state
//  CHECK( gtsam::assert_equal(H1_expectedVel, H1_actualVel, 1e-6));
//  CHECK( gtsam::assert_equal(H2_expectedVel, H2_actualVel, 1e-6));
//  CHECK( gtsam::assert_equal(H3_expectedVel, H3_actualVel, 1e-6));
//  CHECK( gtsam::assert_equal(H4_expectedVel, H4_actualVel, 1e-6));
//  CHECK( gtsam::assert_equal(H5_expectedVel, H5_actualVel, 1e-6));
}



/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
