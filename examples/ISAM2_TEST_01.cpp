#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <vector>
#include <fstream>
#include <time.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;

noiseModel::Diagonal::shared_ptr prior_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished());

//noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());

/* ************************************************************************* */

//const int split_interval = 8; //useless here
const int loop_interval = 8; //16
const int pose_num = 40; //*1

// MH-odometry only. Total 9 hypos in the end
int main(int argc, char* argv[]) {

  clock_t start_time = clock();
  int loop_count = 0;

  ISAM2Params parameters; 
  
  //parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1);  //_initialDelta = 1.0
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0); //_wildfireThreshold = 0.001
  
  parameters.relinearizeThreshold = 0.01;
  //parameters.relinearizeThreshold = 0.0;
  
  parameters.relinearizeSkip = 1;
  ISAM2* isam2 = new ISAM2(parameters); //[2] MHISAM2

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph* graph = new NonlinearFactorGraph(); 

  //mhsiao: Values is expected to handle MHGenericValue directly (maybe not????)
  Values init_values; 
  Values results; 

  Rot3 prior_rotation = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0); 
  Point3 prior_point(Eigen::Vector3d(0.0, 0.0, 0.0));
  
  cout << "== PRE ==" << endl;
  
  //MHGenericValue prior_value(prior_arr[0], mh_isam2->getHypoTreeFirstLayer(), mh_isam2->getHypoTreeRoot());
  Pose3 prior_pose(prior_rotation, prior_point);
  //GenericValue<Pose3> prior_value(prior_pose);
  
  cout << "== PP ==" << endl;
  
  //init_values.insert(X(0), prior_value); //must insert(Key, MHGenericValue)  
  init_values.insert(X(0), prior_pose); //must insert(Key, MHGenericValue)  
  
  cout << "== GV ==" << endl;

  graph->add(PriorFactor<Pose3>(X(0), prior_pose, prior_noise_model));
  
  cout << "== ADD_FACTOR ==" << endl;

  isam2->update(*graph, init_values);
  
  cout << "== UPDATE ==" << endl;
  
  graph->resize(0);
  init_values.clear();
  results = isam2->calculateBestEstimate();
 
  cout << "== CAL_BEST_EST ==" << endl;
  
  Pose3 out_pose = results.at<Pose3>(X(0));
  
  cout << "== READ_RESULT ==" << endl;
  
  //cout << "init: X(0): " << endl << out_pose << endl;

  cout << "== PRINT_RESULT ==" << endl;
  //*
  for (size_t i = 0; i < pose_num; ++i) {
    Eigen::Matrix4d TT = Eigen::Matrix4d::Identity();
    
    /*
    TT(0, 0) = 0.0; 
    TT(0, 1) = -1.0; 
    TT(1, 0) = 1.0; 
    TT(1, 1) = 0.0; 
    // */
    //TODO: choose testing cases
    if (true) { //no else
      Pose3 odom; //
      
      TT(0, 3) = 1.5; 
      odom = Pose3(TT);
      
      //--------------------------------
      Pose3 predict; //
      
      TT(0, 3) = 0.03;
      predict = Pose3(TT);
      
      //--------------------------------
     
      //TODO: odom or predict???? 
      //GenericValue<Pose3> pose_value(results.at<Pose3>(X(i))*odom); //mhsiao: predict!!!
      //GenericValue<Pose3> pose_value(mh_results.at<Pose3>(X(i))*predict); //mhsiao: predict!!!

      //init_values.insert(X(i+1), pose_value); //insert(Key, MHValue)
      init_values.insert(X(i+1), results.at<Pose3>(X(i))*odom); //insert(Key, MHValue)
      
      cout << "== PREDICT_GV ==" << endl;

      //mh_init_values.mhInsert(X(i+1), mh_isam2->predictPose3Multi(dynamic_cast<MHGenericValue<Pose3> >(mh_results.at(X(i))), odom_arr)); //insert(Key, MHValue) //[3] predictMulti(Key, vector<Pose3>)
       
      
      graph->add(BetweenFactor<Pose3>(X(i), X(i + 1), odom, pose_noise_model)); //[4] MHBetweenFactor
      //mh_graph->add(MHFactor2<BetweenFactor<Pose3> >(X(i), X(i + 1), odom_arr, pose_noise_model));
      
      cout << "== ADD_FACTOR ==" << endl;
      // */ 
    }
    
    
    isam2->update(*graph, init_values); //[5]
  
    cout << "== UPDATE ==" << endl;
    
    graph->resize(0);
    init_values.clear();
    results = isam2->calculateBestEstimate(); //[6]
  
    cout << "== CAL_BEST_EST ==" << endl;
    
    /* 
    for (size_t k = 0; k < (i + 2); ++k) {
      Pose3 out_pose = results.at<Pose3>(X(k));
      

      cout << i << ": X(" << k << "): " << endl << out_pose << endl;
      cout << endl;
    }
    cout << "== MH-PRINT_RESULT ==" << endl;
    // */
    
    //================================ LOOP =========================
    if ((i+1)%loop_interval == 0) {
    //if (false) {
      //==== LOOP ====
      loop_count++;

      Pose3 loop_pose; //
      
      TT(0, 3) = 1.5*loop_interval; 
      loop_pose = Pose3(TT);
      
      const int loop_idx = (i+1) - loop_interval; 
      graph->add(BetweenFactor<Pose3>(X(loop_idx), X(i+1), loop_pose, pose_noise_model)); //[4] MHBetweenFactor

    
      isam2->update(*graph); //[5]
      graph->resize(0);
      results = isam2->calculateBestEstimate(); //[6]
      /*
      //TODO: refine result
      for (int lp = 0; lp < 10; ++lp) {
        isam2->update(); //[5]
        results = isam2->calculateBestEstimate(); //[6]
      }
      // */
      /*
      for (size_t k = 0; k < (i + 2); ++k) {
        Pose3 out_pose = results.at<Pose3>(X(k));

        cout << i << ": loop_X(" << k << "): " << endl << out_pose << endl;
        cout << endl;
      }
      cout << "== LOOP-PRINT_RESULT ==" << endl;
      // */
    } // END if LOOP
  }
  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;
  
  //mh_results.print();
  cout << "==== end of test ====" << endl;
  
  ofstream outfile;
  string file_name = "ISAM2_TEST_01";
  outfile.open(file_name + ".txt");
  outfile << "total_time:  " << total_time << endl;
  outfile << "loop_count:  " << loop_count << endl;
  for (size_t i = 0; i < (pose_num + 1); ++i) {
    Pose3 out_pose = results.at<Pose3>(X(i));
    outfile << "out_X(" << i << "): " << endl;
    Vector tt = out_pose.translation().vector();
    outfile << tt(0) << " " << tt(1) << " " << tt(2) << endl;
    outfile << endl;
  }
  outfile.close();
  cout << "output " << file_name << ".txt file." << endl;

  return 0;
}
