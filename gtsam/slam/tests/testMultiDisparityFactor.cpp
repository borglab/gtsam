/*
 * testMultiDisparityFactor.cpp
 *
 *  Created on: Jan 31, 2014
 *      Author: nsrinivasan7
 *      @brief: Unittest for MultidisparityFactor
 */


#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/MultiDisparityFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

void generateDisparities(Eigen::Matrix<double,Eigen::Dynamic,3>& uv, Vector& disparity, Pose3& cameraPose) {

  double w = 640.0;
  double h = 480.0;
  double beta = 0.1;

  double alphax = 700.0;
  double alphay = 700.0;
  double f = (alphax + alphay)/2.0;

  Matrix Rot = cameraPose.rotation().matrix();
  Vector trans = cameraPose.translation().vector();

  // plane parameters
  Matrix norm;
  norm.resize(1,3);
  norm << 1/sqrt(2), 0.0, -1/sqrt(2);
  double d = 20.0;

  uv.resize(w*h,3);

  disparity.resize(w*h);
  for(int u = 0; u < w; u++)
    for(int v = 0; v < h ; v++) {
      uv.row(v*w+u) = Matrix_(1,3, (double)u, (double)v, f*beta);
      Matrix l = norm * trans;
      Matrix disp = ( -1.0/(l(0,0) + d) ) * norm * Rot * ( uv.row(v*w+u).transpose() );

      disparity(v*w+u,0) = disp(0,0);
    }

}

TEST(MutliDisparityFactor,Rd)
{

  Key key(1);
  Vector disparities = Vector_(2, 1.0, 1.0); // matlab generated values

  Eigen::Matrix<double,Eigen::Dynamic,3> uv;
  uv.resize(2,3);
  uv.block<2,3>(0,0) << 20.0, 30.0, 70.0, 40.0, 60.0, 70.0;
  SharedIsotropic model = gtsam::noiseModel::Isotropic::Sigma(disparities.rows(), 0.25, true);

  gtsam::Pose3 cameraPose( gtsam::Rot3(), gtsam::Point3(1.0, 1.0, 1.0) );

  MultiDisparityFactor factor(key, disparities, uv, cameraPose, model);

  // basis = [0 1 0; -1 0 0]
  Vector theta = Vector_(4,0.0,0.0,1.0,20.0);
  OrientedPlane3 p(theta);
  factor.Rd(p);
  Matrix actualRd = factor.Rd();
  Matrix expectedRd = Matrix_(1,4,1.0,1.0,1.0,1.0);
  EXPECT(assert_equal( expectedRd,actualRd,1e-8) );

}

TEST(MutliDisparityFactor,Rn)
{

  Key key(1);
  Vector disparities = Vector_(2, 1.0, 1.0); // matlab generated values

  Eigen::Matrix<double,Eigen::Dynamic,3> uv;
  uv.resize(2,3);
  uv.block<2,3>(0,0) << 20.0, 30.0, 70.0, 40.0, 60.0, 70.0;
  SharedIsotropic model = gtsam::noiseModel::Isotropic::Sigma(disparities.rows(), 0.25, true);

  gtsam::Pose3 cameraPose( gtsam::Rot3(), gtsam::Point3(1.0, 1.0, 1.0) );

  MultiDisparityFactor factor(key, disparities, uv, cameraPose, model);

  Vector theta = Vector_(4,0.0,0.0,1.0,20.0);
  OrientedPlane3 p(theta);
  factor.Rn(p);
  Matrix actualRn = factor.Rn();
  Matrix expectedRn = Matrix_(2,4, -20.0, -30.0, -70.0, 0.0,  -40.0, -60.0, -70.0, 0.0);

  EXPECT(assert_equal( expectedRn,actualRn,1e-8) );
}

// unit test for derivative
TEST(MutliDisparityFactor,H)
{
  Key key(1);
  Vector disparities = Vector_(2, -3.6123, -4.4910); // matlab generated values

  Eigen::Matrix<double,Eigen::Dynamic,3> uv;
  uv.resize(2,3);
  uv.block<2,3>(0,0) << 20.0, 30.0, 70.0, 40.0, 60.0, 70.0;
  SharedIsotropic model = gtsam::noiseModel::Isotropic::Sigma(disparities.rows(), 0.25, true);

  gtsam::Pose3 cameraPose( gtsam::Rot3(), gtsam::Point3(1.0, 1.0, 1.0) );

  MultiDisparityFactor factor(key, disparities, uv, cameraPose, model);

  // basis = [0 1 0; -1 0 0]
  Vector theta = Vector_(4,0.25,1.75,1.0,20.0);
  OrientedPlane3 p(theta);

  Matrix actualH;
  factor.R(p);

  Vector theta1 = Vector_(4,0.45,0.45,1.0,20.0);
  OrientedPlane3 p1(theta1);

  Vector err = factor.evaluateError(p1,actualH);

  Matrix expectedH = numericalDerivative11<OrientedPlane3>(
      boost::bind(&MultiDisparityFactor::evaluateError, &factor, _1, boost::none), p1);

  EXPECT(assert_equal( expectedH,actualH,1e-8) );
}

// unit test for optimization
TEST(MultiDisparityFactor,optimize) {

  NonlinearFactorGraph graph;

  Vector disparities;
  Eigen::Matrix<double,Eigen::Dynamic,3> uv;

  gtsam::Rot3 R = gtsam::Rot3();
  gtsam::Pose3 cameraPose( R.RzRyRx(0,-M_PI/3,0) , gtsam::Point3(50.0, 0.0, 50.0) );

  generateDisparities(uv,disparities,cameraPose);

  Key key(1);
  SharedIsotropic model = gtsam::noiseModel::Isotropic::Sigma(disparities.rows(), 0.25, true);
  MultiDisparityFactor factor1(key, disparities, uv, cameraPose, model);
  graph.push_back(factor1);

  Values initialEstimate;
  initialEstimate.insert(1, OrientedPlane3( 1.0/sqrt(2) + 0.2, 0.3, -1.0/sqrt(2) - 0.2, 20.0 ) );

  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 1000;
  // Create the optimizer ...
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // ... and optimize
  Values actualresult = optimizer.optimize();

  Values expectedresult;
  expectedresult.insert(1, OrientedPlane3( 1.0/sqrt(2), 0.0, -1.0/sqrt(2), 20.0 ) );

  EXPECT(assert_equal( expectedresult,actualresult,1e-8) );
}

// model selection test with two models
TEST(MultiDisparityFactor,modelselect)
{

  //  ************************Image 1
  Vector disparities1;
  Eigen::Matrix<double,Eigen::Dynamic,3> uv1;

  gtsam::Rot3 R1 = gtsam::Rot3();
  gtsam::Pose3 cameraPose1( R1.RzRyRx(0,-M_PI/3,0) , gtsam::Point3(50.0, 0.0, 50.0) );

  generateDisparities(uv1,disparities1,cameraPose1);

  // ***************************Image 2
  NonlinearFactorGraph graph2;

  Vector disparities2;
  Eigen::Matrix<double,Eigen::Dynamic,3> uv2;

  gtsam::Rot3 R2 = gtsam::Rot3();
  gtsam::Pose3 cameraPose2( R2.RzRyRx(0,-M_PI/4,0) , gtsam::Point3(30.0, 0.0, 20.0) );

  generateDisparities(uv2,disparities2,cameraPose2);

  // ****************************Model 1

  NonlinearFactorGraph graph1;
  Key key1(1);
  SharedIsotropic model1 = gtsam::noiseModel::Isotropic::Sigma(disparities1.rows(), 0.25, true);
  MultiDisparityFactor factor1(key1, disparities1, uv1, cameraPose1, model1);
  graph1.push_back(factor1);

  Values initialEstimate1;
  initialEstimate1.insert(1, OrientedPlane3( 1.0/sqrt(2) + 0.2, 0.3, -1.0/sqrt(2) - 0.2, 20.0 ) );

  GaussNewtonParams parameters1;
  // Stop iterating once the change in error between steps is less than this value
  parameters1.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters1.maxIterations = 1000;
  // Create the optimizer ...
  GaussNewtonOptimizer optimizer1(graph1, initialEstimate1, parameters1);
  // ... and optimize
  Values result1 = optimizer1.optimize();

  Marginals marginals1(graph1, result1);
  print(marginals1.marginalCovariance(1), "Theta1 Covariance");

  // ****************************Model 2

//  Key key2(1);
//  SharedIsotropic model2 = gtsam::noiseModel::Isotropic::Sigma(disparities2.rows(), 0.25, true);
//  MultiDisparityFactor factor2(key2, disparities2, uv2, cameraPose2, model2);
//  graph2.push_back(factor2);
//
//  Values initialEstimate2;
//  initialEstimate2.insert(1, OrientedPlane3( 1.0/sqrt(2) + 0.2, 0.3, -1.0/sqrt(2) - 0.2, 20.0 ) );
//
//  GaussNewtonParams parameters2;
//  // Stop iterating once the change in error between steps is less than this value
//  parameters2.relativeErrorTol = 1e-5;
//  // Do not perform more than N iteration steps
//  parameters2.maxIterations = 1000;
//  // Create the optimizer ...
//  GaussNewtonOptimizer optimizer2(graph2, initialEstimate2, parameters2);
//  // ... and optimize
//  Values actualresult2 = optimizer2.optimize();
//
//  Values expectedresult2;
//  expectedresult2.insert(1, OrientedPlane3( 1.0/sqrt(2), 0.0, -1.0/sqrt(2), 20.0 ) );
//
//  Values result2 = optimizer2.optimize();
//
//  Marginals marginals2(graph2, result2);
//  print(marginals2.marginalCovariance(2), "Theta2 Covariance");

}
/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
