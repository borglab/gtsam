/*
 * testInvDepthFactorVariant3.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: cbeall3
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant3.h>
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
TEST( InvDepthFactorVariant3, optimize) {

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
  Point3 landmark_p1 = pose1.transformTo(landmark);
  // landmark_p1.print("Landmark in Pose1 Frame:\n");
  double theta = atan2(landmark_p1.x(), landmark_p1.z());
  double phi = atan2(landmark_p1.y(), sqrt(landmark_p1.x()*landmark_p1.x()+landmark_p1.z()*landmark_p1.z()));
  double rho = 1./landmark_p1.norm();
  Vector3 expected((Vector(3) << theta, phi, rho).finished());



  // Create a factor graph with two inverse depth factors and two pose priors
  Key poseKey1(1);
  Key poseKey2(2);
  Key landmarkKey(100);
  SharedNoiseModel sigma(noiseModel::Unit::Create(2));
  NonlinearFactor::shared_ptr factor1(new NonlinearEquality<Pose3>(poseKey1, pose1, 100000));
  NonlinearFactor::shared_ptr factor2(new NonlinearEquality<Pose3>(poseKey2, pose2, 100000));
  NonlinearFactor::shared_ptr factor3(new InvDepthFactorVariant3a(poseKey1, landmarkKey, pixel1, K, sigma));
  NonlinearFactor::shared_ptr factor4(new InvDepthFactorVariant3b(poseKey1, poseKey2, landmarkKey, pixel2, K, sigma));
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



  // Test that the correct landmark parameters have been recovered
  EXPECT(assert_equal((Vector)expected, actual, 1e-9));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
