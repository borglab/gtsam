/**
 * @file    Pose3SLAMExampleExpressions_BearingRangeWithTransform.cpp
 * @brief   A simultaneous optimization of trajectory, landmarks and sensor-pose with respect to body-pose using bearing-range measurements done with Expressions
 * @author  Thomas Horstink
 * @date    January 4th, 2019
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <examples/SFMdata.h>

using namespace gtsam;

typedef BearingRange<Pose3, Point3> BearingRange3D;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Move around so the whole state (including the sensor tf) is observable
  Pose3 init_pose = Pose3();
  Pose3 delta_pose1 = Pose3(Rot3().Yaw(2*M_PI/8).Pitch(M_PI/8), Point3(1, 0, 0));
  Pose3 delta_pose2 = Pose3(Rot3().Pitch(-M_PI/8), Point3(1, 0, 0));
  Pose3 delta_pose3 = Pose3(Rot3().Yaw(-2*M_PI/8), Point3(1, 0, 0));

  int steps = 4;
  auto poses  = createPoses(init_pose, delta_pose1, steps);
  auto poses2 = createPoses(init_pose, delta_pose2, steps);
  auto poses3 = createPoses(init_pose, delta_pose3, steps);

  // Concatenate poses to create trajectory
  poses.insert( poses.end(), poses2.begin(), poses2.end() );
  poses.insert( poses.end(), poses3.begin(), poses3.end() );  // std::vector of Pose3
  auto points = createPoints();                               // std::vector of Point3

  // (ground-truth) sensor pose in body frame, further an unknown variable
  Pose3 body_T_sensor_gt(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  // The graph
  ExpressionFactorGraph graph;

  // Specify uncertainty on first pose prior and also for between factor (simplicity reasons)
  auto poseNoise = noiseModel::Diagonal::Sigmas((Vector(6)<<0.3,0.3,0.3,0.1,0.1,0.1).finished());

  // Uncertainty bearing range measurement;
  auto bearingRangeNoise = noiseModel::Diagonal::Sigmas((Vector(3)<<0.01,0.03,0.05).finished());

  // Expressions for body-frame at key 0 and sensor-tf
  Pose3_ x_('x', 0);
  Pose3_ body_T_sensor_('T', 0);

  // Add a prior on the body-pose
  graph.addExpressionFactor(x_, poses[0], poseNoise);

  // Simulated measurements from pose
  for (size_t i = 0; i < poses.size(); ++i) {
    auto world_T_sensor = poses[i].compose(body_T_sensor_gt);
    for (size_t j = 0; j < points.size(); ++j) {

      // This expression is the key feature of this example: it creates a differentiable expression of the measurement after being displaced by sensor transform.
      auto prediction_ = Expression<BearingRange3D>( BearingRange3D::Measure, Pose3_('x',i)*body_T_sensor_, Point3_('l',j));

      // Create a *perfect* measurement
      auto measurement = BearingRange3D(world_T_sensor.bearing(points[j]), world_T_sensor.range(points[j]));

      // Add factor
      graph.addExpressionFactor(prediction_, measurement, bearingRangeNoise);
    }

    // and add a between factor to the graph
    if (i > 0)
    {
      // And also we have a *perfect* measurement for the between factor.
      graph.addExpressionFactor(between(Pose3_('x', i-1),Pose3_('x', i)), poses[i-1].between(poses[i]), poseNoise);
    }
  }

  // Create perturbed initial
  Values initial;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < poses.size(); ++i)
    initial.insert(Symbol('x', i), poses[i].compose(delta));
  for (size_t j = 0; j < points.size(); ++j)
    initial.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));

  // Initialize body_T_sensor wrongly (because we do not know!)
  initial.insert<Pose3>(Symbol('T',0), Pose3());

  std::cout << "initial error: " << graph.error(initial) << std::endl;
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  std::cout << "final error: " << graph.error(result) << std::endl;

  initial.at<Pose3>(Symbol('T',0)).print("\nInitial estimate body_T_sensor\n"); /* initial sensor_P_body estimate */
  result.at<Pose3>(Symbol('T',0)).print("\nFinal estimate body_T_sensor\n");    /* optimized sensor_P_body estimate */
  body_T_sensor_gt.print("\nGround truth body_T_sensor\n");                     /* sensor_P_body ground truth */

  return 0;
}
/* ************************************************************************* */