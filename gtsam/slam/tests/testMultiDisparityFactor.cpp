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
  Matrix expectedRd = Matrix_(1,3,1.0,-1.0,0.0);
//  EXPECT(assert_equal( expectedRd,actualRd,1e-8) );

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

  // basis = [0 1 0; -1 0 0]
  Vector theta = Vector_(4,0.0,0.0,1.0,20.0);
  OrientedPlane3 p(theta);
  factor.Rn(p);
  Matrix actualRn = factor.Rn();
  Matrix expectedRn = Matrix_(2,3, 30.0, -20.0, 0.0, 60.0, -40.0, 0.0);

//  EXPECT(assert_equal( expectedRn,actualRn,1e-8) );
}

TEST(MutliDisparityFactor,R)
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
  factor.Rn(p);
  factor.Rd(p);
  factor.R(p);

  Matrix expectedR;
  expectedR.resize(3,3);
  expectedR <<  0,    10,     0,
               -10,     0,     0,
                 0,     0,     0;
  Matrix actualR = factor.getR(0);

//  EXPECT(assert_equal( expectedR,actualR,1e-8) );
  expectedR <<  0,    20,     0,
                 -20,     0,     0,
                   0,     0,     0;

  actualR = factor.getR(1);
//  EXPECT(assert_equal( expectedR,actualR,1e-8) );
}

TEST(MutliDisparityFactor,H)
{
  Key key(1);
  Vector disparities = Vector_(2, 20.0, 40.0); // matlab generated values

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

  cout << "expectedH :" << expectedH << "\n";
  cout << "actualH :" << actualH << "\n";
//  EXPECT(assert_equal( expectedH,actualH,1e-8) );
}

/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
