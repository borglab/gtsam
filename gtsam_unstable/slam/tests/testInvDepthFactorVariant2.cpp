/*
 * testInvDepthFactorVariant1.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: cbeall3
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant2.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( InvDepthFactorVariant2, optimize) {

  // Create two poses looking in the x-direction
  Pose3 pose1(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1.0));
  Pose3 pose2(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1.5));

  // Create a landmark 5 meters in front of pose1 (camera center at (0,0,1))
  Point3 landmark(5, 0, 1);

  // Create image observations
  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  PinholeCamera<Cal3_S2> camera1(pose1, *K);
  PinholeCamera<Cal3_S2> camera2(pose2, *K);
  Point2 pixel1 = camera1.project(landmark);
  Point2 pixel2 = camera2.project(landmark);

  // Create expected landmark
  Point3 referencePoint = pose1.translation();
  Point3 ray = landmark - referencePoint;
  double theta = atan2(ray.y(), ray.x());
  double phi = atan2(ray.z(), sqrt(ray.x()*ray.x()+ray.y()*ray.y()));
  double rho = 1./ray.norm();
  Vector3 expected((Vector(3) << theta, phi, rho).finished());



  // Create a factor graph with two inverse depth factors and two pose priors
  Key poseKey1(1);
  Key poseKey2(2);
  Key landmarkKey(100);
  SharedNoiseModel sigma(noiseModel::Unit::Create(2));
  NonlinearFactor::shared_ptr factor1(new NonlinearEquality<Pose3>(poseKey1, pose1, 100000));
  NonlinearFactor::shared_ptr factor2(new NonlinearEquality<Pose3>(poseKey2, pose2, 100000));
  NonlinearFactor::shared_ptr factor3(new InvDepthFactorVariant2(poseKey1, landmarkKey, pixel1, K, referencePoint, sigma));
  NonlinearFactor::shared_ptr factor4(new InvDepthFactorVariant2(poseKey2, landmarkKey, pixel2, K, referencePoint, sigma));
  NonlinearFactorGraph graph;
  graph.push_back(factor1);
  graph.push_back(factor2);
  graph.push_back(factor3);
  graph.push_back(factor4);

  // Create a values with slightly incorrect initial conditions
  Values values;
  values.insert(poseKey1, pose1.retract((Vector(6) << +0.01, -0.02, +0.03, -0.10, +0.20, -0.30).finished()));
  values.insert(poseKey2, pose2.retract((Vector(6) << +0.01, +0.02, -0.03, -0.10, +0.20, +0.30).finished()));
  values.insert(landmarkKey, Vector3(expected + Vector3(+0.02, -0.04, +0.05)));

  // Optimize the graph to recover the actual landmark position
  LevenbergMarquardtParams params;
  Values result = LevenbergMarquardtOptimizer(graph, values, params).optimize();
  Vector3 actual = result.at<Vector3>(landmarkKey);

//  values.at<Pose3>(poseKey1).print("Pose1 Before:\n");
//  result.at<Pose3>(poseKey1).print("Pose1 After:\n");
//  pose1.print("Pose1 Truth:\n");
//  std::cout << std::endl << std::endl;
//  values.at<Pose3>(poseKey2).print("Pose2 Before:\n");
//  result.at<Pose3>(poseKey2).print("Pose2 After:\n");
//  pose2.print("Pose2 Truth:\n");
//  std::cout << std::endl << std::endl;
//  values.at<Vector3>(landmarkKey).print("Landmark Before:\n");
//  result.at<Vector3>(landmarkKey).print("Landmark After:\n");
//  expected.print("Landmark Truth:\n");
//  std::cout << std::endl << std::endl;

  // Calculate world coordinates of landmark versions
  Point3 world_landmarkBefore(0,0,0);
  {
    Vector3 landmarkBefore = values.at<Vector3>(landmarkKey);
    double theta = landmarkBefore(0), phi = landmarkBefore(1), rho = landmarkBefore(2);
    world_landmarkBefore = referencePoint + Point3(cos(theta)*cos(phi)/rho, sin(theta)*cos(phi)/rho, sin(phi)/rho);
  }
  Point3 world_landmarkAfter(0,0,0);
  {
    Vector3 landmarkAfter = result.at<Vector3>(landmarkKey);
    double theta = landmarkAfter(0), phi = landmarkAfter(1), rho = landmarkAfter(2);
    world_landmarkAfter = referencePoint + Point3(cos(theta)*cos(phi)/rho, sin(theta)*cos(phi)/rho, sin(phi)/rho);
  }

//  world_landmarkBefore.print("World Landmark Before:\n");
//  world_landmarkAfter.print("World Landmark After:\n");
//  landmark.print("World Landmark Truth:\n");
//  std::cout << std::endl << std::endl;

  // Test that the correct landmark parameters have been recovered
  EXPECT(assert_equal((Vector)expected, actual, 1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
