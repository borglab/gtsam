#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::P;
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.001, 0.001, 0.001).finished());
noiseModel::Diagonal::shared_ptr br_noise_model = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2).finished());

// 2D SLAM with multi-hypo bearing-range measurements

int main(int argc, char** argv) {

  ISAM2Params parameters;
  parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1);
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2* isam2 = new ISAM2(parameters);

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph* graph = new NonlinearFactorGraph();
  Values init_values;
  Values results;
  
  Pose2 prior_pose(0.0, 0.0, 0.0);

  graph->add(PriorFactor<gtsam::Pose2>(X(0), prior_pose, pose_noise_model));
  
  init_values.insert(X(0), prior_pose);
  isam2->update(*graph, init_values);
  graph->resize(0);
  init_values.clear();
  results = isam2->calculateBestEstimate();


  size_t pose_num = 4;
  for (int i = 0; i < pose_num; ++i) {
    double theta = 0.0;
    Eigen::Matrix2d RR = Rot2(theta).matrix();
    Eigen::Vector2d tt = Eigen::Vector2d::Zero();
    tt(0, 0) = 2.0;
    
    Eigen::Matrix2d last_RR = results.at<Pose2>(X(i)).r().matrix(); 
    double last_theta = results.at<Pose2>(X(i)).r().theta(); 
    Eigen::Vector2d last_tt = results.at<Pose2>(X(i)).t().vector(); 
    Pose2 predict_pose = Pose2(Rot2(last_theta + theta), Point2(last_tt + last_RR*tt));
    init_values.insert(X(i+1), predict_pose);
    
    Pose2 odom = Pose2(Rot2(theta), Point2(tt));
    graph->add(BetweenFactor<Pose2>(X(i), X(i + 1), odom, pose_noise_model));
   
    if (i == 0) {
        Rot2 b_1 = Rot2::fromDegrees(90);
        double r_1 = 2.0;
        Point2 local_point = Point2(r_1*b_1.c(), r_1*b_1.s());
        Point2 predict_point = Point2((predict_pose.r().rotate(local_point)).vector() + predict_pose.t().vector());

        init_values.insert(P(0), predict_point);

        graph->add(BearingRangeFactor<Pose2, Point2>(X(1), P(0), b_1, r_1, br_noise_model));
    } else if (i == 1) {
        Rot2 b_2 = Rot2::fromDegrees(45);
        double r_2 = std::sqrt(4.0 + 4.0);
        Point2 local_point = Point2(r_2*b_2.c(), r_2*b_2.s());
        Point2 predict_point = Point2((predict_pose.r().rotate(local_point)).vector() + predict_pose.t().vector());
        
        init_values.insert(P(1), predict_point);

        graph->add(BearingRangeFactor<Pose2, Point2>(X(2), P(1), b_2, r_2, br_noise_model));
    } else if (i == 2) {
        // Make this a MHFactor
        Rot2 b_3 = Rot2::fromDegrees(90);
        double r_3 = 2.0;
        std::list<Key> P_list;
        P_list.push_back(P(0));
        P_list.push_back(P(1));
        graph->add(MHBearingRangeFactor<Pose2, Point2>(X(3), P_list, b_3, r_3, br_noise_model)); //[8] //different Keys, same measured
    }
    
    isam2->update(*graph, init_values);
    graph->resize(0);
    init_values.clear();
    results = isam2->calculateBestEstimate();
  }
  results.print();
  /*
  // create the measurement values - indices are (pose id, landmark id)
  Rot2 bearing11 = Rot2::fromDegrees(45),
       bearing21 = Rot2::fromDegrees(90),
       bearing32 = Rot2::fromDegrees(90);
  double range11 = std::sqrt(4.0+4.0),
         range21 = 2.0,
         range32 = 2.0;

  // Add Bearing-Range factors
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x1, l1, bearing11, range11, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x2, l1, bearing21, range21, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x3, l2, bearing32, range32, measurementNoise);
  // */

  return 0;
}

