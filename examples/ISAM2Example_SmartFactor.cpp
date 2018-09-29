#include <iostream>
#include <vector>

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

// Make the typename short so it looks much cleaner
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

int main(int argc, char* argv[]) {
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  noiseModel::Isotropic::shared_ptr measurement_noise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  noiseModel::Diagonal::shared_ptr noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
  noiseModel::Isotropic::shared_ptr large_noise = noiseModel::Isotropic::Sigma(6, 100);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  parameters.print();
  ISAM2 isam(parameters);

  // Create a factor graph
  NonlinearFactorGraph graph;
  Values initial_estimate;

  Point3 point(0.0, 0.0, 1.0);

  // Intentionally initialize the variables off from the ground truth
  Pose3 delta(Rot3::Rodrigues(0.0, 0.0, 0.0), Point3(0.05, -0.10, 0.20));

  Pose3 pose1(Rot3(), Point3(0.0, 0.0, 0.0));
  Pose3 pose2(Rot3(), Point3(0.0, 0.2, 0.0));
  Pose3 pose3(Rot3(), Point3(0.0, 0.4, 0.0));
  Pose3 pose4(Rot3(), Point3(0.0, 0.5, 0.0));
  Pose3 pose5(Rot3(), Point3(0.0, 0.6, 0.0));

  std::vector<Pose3> pose_list = { pose1, pose2, pose3, pose4, pose5 };

  SmartFactor::shared_ptr smart_factor(new SmartFactor(measurement_noise, K));
  graph.push_back(smart_factor);

  graph.emplace_shared<PriorFactor<Pose3>>(0, pose_list[0], noise);
  initial_estimate.insert(0, pose_list[0].compose(delta));
  smart_factor->add(PinholePose<Cal3_S2>(pose_list[0], K).project(point), 0);

  for (int i = 1; i < 5; i++) {
    PinholePose<Cal3_S2> camera(pose_list[i], K);
    Point2 measurement = camera.project(point);

    std::cout << "****************************************************" << std::endl;
    std::cout << "i = " << i << std::endl;
    std::cout << "Measurement " << i << " is " << measurement << std::endl;

    graph.emplace_shared<PriorFactor<Pose3>>(i, pose_list[i], noise);
    //graph.emplace_shared<BetweenFactor<Pose3>>(i - 1, i, Pose3(), large_noise);
    initial_estimate.insert(i, pose_list[i].compose(delta));
    smart_factor->add(measurement, i);

    ISAM2Result result = isam.update(graph, initial_estimate);
    graph.resize(0);
    initial_estimate.clear();

    result.print();

    const auto& var_map = (*(result.detail)).variableStatus;

    std::cout << "Detailed results:" << std::endl;
    for (int j = 0; j < 3; j++) {
      if (var_map.exists(j)) {
        std::cout << j << " is reeliminated: " << var_map.at(j).isReeliminated << std::endl;
        std::cout << j << " is relinearized above thresh: " << var_map.at(j).isAboveRelinThreshold << std::endl;
        std::cout << j << " is relinearized involved: " << var_map.at(j).isRelinearizeInvolved << std::endl;
        std::cout << j << " is relinearized: " << var_map.at(j).isRelinearized << std::endl;
        std::cout << j << " is observed: " << var_map.at(j).isObserved << std::endl;
        std::cout << j << " is new: " << var_map.at(j).isNew << std::endl;
        std::cout << j << " is in the root clique: " << var_map.at(j).inRootClique << std::endl;
      }
      else {
        std::cout << j << " does not exist in the detailed results map." << std::endl;
      }
    }

    Values current_estimate = isam.calculateEstimate();
    current_estimate.print("Current estimate: ");

    boost::optional<Point3> point_res = smart_factor->point(current_estimate);
    if (point_res) {
      std::cout << *point_res << std::endl;
    }
    else {
      std::cout << "Point is degenerate." << std::endl;
    }
  }

  return 0;
}
