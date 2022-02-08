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
// MH-loop-closing at the end. Still pose only.

int main(int argc, char* argv[]) {

  MHISAM2Params parameters; //[1] MHISAM2Params
  parameters.optimizationParams = gtsam::MHISAM2DoglegParams(0.1); //[2] MHISAM2DoglegParams
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  MHISAM2* mh_isam2 = new MHISAM2(parameters); //[3] MHISAM2

  // Create a Factor Graph and Values to hold the new data
  MHNonlinearFactorGraph* mh_graph = new MHNonlinearFactorGraph(); //[4]MHNonlinearFactorGraph
  MHValues mh_init_values; //[5] MHValues
  MHValues mh_results; 

  Rot3 prior_rotation = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0); 
  Point3 prior_point(Eigen::Vector3d(0.0, 0.0, 0.0));
  Pose3 prior_pose(prior_rotation, prior_point);

  mh_graph->add(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
  
  mh_init_values.insert(X(0), prior_pose);
  mh_isam2->update(*mh_graph, mh_init_values);
  mh_graph->resize(0);
  mh_init_values.clear();
  mh_results = mh_isam2->calculateBestEstimate();

  size_t pose_num = 4;
  for (int i = 0; i < pose_num; ++i) {
    Eigen::Matrix4d TT = Eigen::Matrix4d::Identity();
    TT(0, 3) = 1.0*(i+1); 
    

    Pose3 odom = Pose3(TT);
    mh_init_values.insert(X(i+1), mh_isam2->predictSingle(X(i), odom)); //

    mh_graph->add(BetweenFactor<Pose3>(X(i), X(i + 1), odom, pose_noise_model)); 

    if (i == (pose_num - 1)) { //last pose
    
    	Eigen::Matrix4d loop_TT = Eigen::Matrix4d::Identity();
    	loop_TT(0, 3) = 10;
	Pose3 loop = Pose3(loop_TT);
    	
	mh_graph->add(MHLoopFactor<Pose3>(X(i), X(0), loop, pose_noise_model)); //[7] //can be a link or not. Total 2 hypos
    }


    mh_isam2->update(*mh_graph, mh_init_values);
    mh_graph->resize(0);
    mh_init_values.clear();
    mh_results = mh_isam2->calculateBestEstimate();
  }
  mh_results.print();
  return 0;
}


