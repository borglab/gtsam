#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <vector>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
/* ************************************************************************* */

// MH-odometry only. Total 9 hypos in the end
int main(int argc, char* argv[]) {

  ISAM2Params parameters; 
  MHParams mh_parameters; //[1] MHParams
  parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1); 
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  MHISAM2* mh_isam2 = new MHISAM2(parameters, mh_parameters); //[2] MHISAM2

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph* mh_graph = new NonlinearFactorGraph(); 
  Values mh_init_values; 
  Values mh_results; 

  Rot3 prior_rotation = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0); 
  Point3 prior_point(Eigen::Vector3d(0.0, 0.0, 0.0));
  Pose3 prior_pose(prior_rotation, prior_point);
  
  MHGenericValue prior_value(prior_pose, mh_isam2->getHypoTreeRoot());

  mh_graph->add(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
  
  mh_init_values.insert(X(0), prior_value); //must insert(Key, MHGenericValue) 
  mh_isam2->update(*mh_graph, mh_init_values);
  mh_graph->resize(0);
  mh_init_values.clear();
  mh_results = mh_isam2->calculateBestEstimate();

  size_t pose_num = 4;
  for (size_t i = 0; i < pose_num; ++i) {
    Eigen::Matrix4d TT = Eigen::Matrix4d::Identity();
    TT(0, 3) = 1.0*(i+1); 
    
    if (i%2 == 0) {
      std::list<Pose3> odom_list; //
      
      TT_list.push_back(TT);
      odom_list.push_back(Pose3(TT));
      TT(1, 3) = 0.1*(i+1);
      TT_list.push_back(TT);
      odom_list.push_back(Pose3(TT));
      TT(1, 3) = 0.15*(i+1);
      TT_list.push_back(TT);
      odom_list.push_back(Pose3(TT));
      
      /* 
      std::list<Pose3> predict_pose_list;
      for (int h = 0; h < hypo_num(X(i)); ++h) {
        for (std::list<Eigen::Matrix4d>::iterator it = TT_list.begin(); it != TT_list.end(); ++it) {
          //TODO should consider hypo links
          predict_pose_list.push_back(Pose3(mh_results.at<Pose3>(X(i), h).matrix()*(*it));
        }
      }
      mh_init_values.insert(X(i+1), predict_pose_list);
      // */
      
      
      mh_init_values.insert(X(i+1), mh_isam2->predictPose3Multi(mh_results.at(X(i)), odom_list)); //insert(Key, MHValue) //[3] predictMulti(Key, list<Pose3>)

      mh_graph->add(MHBetweenFactor<Pose3>(X(i), X(i + 1), odom_list, pose_noise_model)); //[4] MHBetweenFactor
      //mh_graph->add(MHFactor2<BetweenFactor<Pose3> >(X(i), X(i + 1), odom_list, pose_noise_model));
      
      mh_isam2->assocLatestHypoLayerWith(mh_graph->back()); //mhiso: only do this when a new MH-NonlinearFactor is added
      
       
    } else {

      Pose3 odom = Pose3(TT);
      /* 
      std::list<Pose3> predict_pose_list;
      for (int h = 0; h < ; ++h) {
        predict_pose_list.push_back(Pose3(mh_results.at<Pose3>(X(i), h).matrix()*TT);
      }
      mh_init_values.insert(X(i+1), predict_pose_list);
      // */
      mh_init_values.insert(X(i+1), mh_isam2->predictPose3Single(mh_results.at(X(i)), odom)); //

      mh_graph->add(BetweenFactor<Pose3>(X(i), X(i + 1), odom, pose_noise_model)); 

    }

    mh_isam2->update(*mh_graph, mh_init_values); //[5]
    mh_graph->resize(0);
    mh_init_values.clear();
    mh_results = mh_isam2->calculateBestEstimate(); //[6]
  }
  mh_results.print();
  return 0;
}


