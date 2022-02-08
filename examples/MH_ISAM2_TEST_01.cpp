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

const int split_interval = 3; //2
const int loop_interval = 4; //16
const int pose_num = 20; //*1

// MH-odometry only. Total 9 hypos in the end
int main(int argc, char* argv[]) {

  clock_t start_time = clock();  
  int loop_count = 0;

  ISAM2Params parameters; 
  MHParams mh_parameters; //[1] MHParams
  
  //parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1);  //_initialDelta = 1.0
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0); //_wildfireThreshold = 0.001
  
  parameters.relinearizeThreshold = 0.01;
  //parameters.relinearizeThreshold = 0.0;
  
  parameters.relinearizeSkip = 1;
  MHISAM2* mh_isam2 = new MHISAM2(parameters, mh_parameters); //[2] MHISAM2

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph* mh_graph = new NonlinearFactorGraph(); 

  //mhsiao: Values is expected to handle MHGenericValue directly (maybe not????)
  Values mh_init_values; 
  Values mh_results; 

  Rot3 prior_rotation = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0); 
  Point3 prior_point(Eigen::Vector3d(0.0, 0.0, 0.0));
  std::vector<Pose3> prior_arr;
  prior_arr.push_back(Pose3(prior_rotation, prior_point));
  
  cout << "== PRE ==" << endl;
  
  //MHGenericValue prior_value(prior_arr[0], mh_isam2->getHypoTreeFirstLayer(), mh_isam2->getHypoTreeRoot());
  MHGenericValue<Pose3> prior_value(prior_arr[0], mh_isam2->getFirstHypoLayer() );
  
  cout << "== MHPP ==" << endl;
  
  mh_init_values.mhInsert(X(0), prior_value); //must insert(Key, MHGenericValue)  
  
  cout << "== MHGV ==" << endl;

  mh_graph->add(MHPriorFactor<Pose3>(X(0), prior_arr, prior_noise_model));
  
  cout << "== ADD_FACTOR ==" << endl;

  mh_isam2->update(*mh_graph, mh_init_values);
  
  cout << "== UPDATE ==" << endl;
  
  mh_graph->resize(0);
  mh_init_values.clear();
  mh_results = mh_isam2->mhCalculateBestEstimate();
 
  cout << "== CAL_BEST_EST ==" << endl;
  
  vector<Pose3> out_arr = mh_results.mhAt<Pose3>(X(0));
  
  cout << "== READ_RESULT ==" << endl;
  
  //cout << "init: X(0): " << endl << out_arr.front() << endl;

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
    if ((i+1)%split_interval == 0) {
    //if (true) {
    //if (false) {
      std::vector<Pose3> odom_arr; //
      
      TT(0, 3) = 1.5; 
      odom_arr.push_back(Pose3(TT));
      
      TT(0, 3) = 2.0;
      odom_arr.push_back(Pose3(TT));
      
      /*
      TT(1, 3) = 2.0*(i+1);
      odom_arr.push_back(Pose3(TT));
      // */
      //--------------------------------
      std::vector<Pose3> predict_arr; //
      
      TT(0, 3) = 1.5;
      predict_arr.push_back(Pose3(TT));
      
      TT(0, 3) = 2.0;
      predict_arr.push_back(Pose3(TT));
      
      /* 
      TT(1, 3) = 2.0*(i+1) + 0.07;
      predict_arr.push_back(Pose3(TT));
      // */
      //--------------------------------

      mh_isam2->createNextHypoLayer(odom_arr.size(), 6); //mhsiao: NOTICE: new variable must exyend from the old one that is in the current last HypoLayer
     
      //TODO: odom or predict???? 
      MHGenericValue<Pose3> pose_value(mh_results.mhAt<Pose3>(X(i)), odom_arr, mh_isam2->getLastHypoLayer() ); //mhsiao: predict!!!
      //MHGenericValue<Pose3> pose_value(mh_results.mhAt<Pose3>(X(i)), predict_arr, mh_isam2->getLastHypoLayer(), mh_isam2->getLastNodeList() ); //mhsiao: predict!!!

      mh_init_values.mhInsert(X(i+1), pose_value); //insert(Key, MHValue)

      //mh_init_values.mhInsert(X(i+1), mh_isam2->predictPose3Multi(dynamic_cast<MHGenericValue<Pose3> >(mh_results.at(X(i))), odom_arr)); //insert(Key, MHValue) //[3] predictMulti(Key, vector<Pose3>)
       
      cout << "== MH-PREDICT_MHGV ==" << endl;
      
      
      mh_graph->add(MHBetweenFactor<Pose3>(X(i), X(i + 1), odom_arr, pose_noise_model)); //[4] MHBetweenFactor
      //mh_graph->add(MHFactor2<BetweenFactor<Pose3> >(X(i), X(i + 1), odom_arr, pose_noise_model));

      cout << "== MH-ADD_FACTOR ==" << endl;
      
      mh_isam2->assocLatestHypoLayerWith(mh_graph->back()); //mhiso: only do this when a new MH-NonlinearFactor is added
      
      //cout << "== MH-ASSOC_LAYER ==" << endl;
      // */ 
       
    } else {

      /*
      Pose3 odom = Pose3(TT);
      
      mh_init_values.mhInsert(X(i+1), mh_isam2->predictPose3Single(mh_results.at(X(i)), odom)); //

      mh_graph->add(BetweenFactor<Pose3>(X(i), X(i + 1), odom, pose_noise_model)); 
      // */
      std::vector<Pose3> odom_arr; //
      
      TT(0, 3) = 1.5; 
      odom_arr.push_back(Pose3(TT));
      
      //mh_isam2->createNextHypoLayer(odom_arr.size()); //mhsiao: NOTICE: new variable must extend from the old one that is in the current last HypoLayer
      
      MHGenericValue<Pose3> pose_value(mh_results.mhAt<Pose3>(X(i)), odom_arr, mh_isam2->getLastHypoLayer() ); //mhsiao: predict!!!

      mh_init_values.mhInsert(X(i+1), pose_value); //insert(Key, MHValue)

      //mh_init_values.mhInsert(X(i+1), mh_isam2->predictPose3Multi(dynamic_cast<MHGenericValue<Pose3> >(mh_results.at(X(i))), odom_arr)); //insert(Key, MHValue) //[3] predictMulti(Key, vector<Pose3>)
       
      cout << "== SG-PREDICT_MHGV ==" << endl;
      
      
      mh_graph->add(MHBetweenFactor<Pose3>(X(i), X(i + 1), odom_arr, pose_noise_model)); //[4] MHBetweenFactor
      //mh_graph->add(MHFactor2<BetweenFactor<Pose3> >(X(i), X(i + 1), odom_arr, pose_noise_model));

      cout << "== SG-ADD_FACTOR ==" << endl;
      
      //mh_isam2->assocLatestHypoLayerWith(mh_graph->back()); //mhiso: only do this when a new MH-NonlinearFactor is added
      
      //cout << "== SG-ASSOC_LAYER ==" << endl;
      // */ 
    }
    
    
    mh_isam2->update(*mh_graph, mh_init_values); //[5]
  
    cout << "== MH-UPDATE ==" << endl;
    
    //*
    mh_graph->resize(0);
    mh_init_values.clear();
    mh_results = mh_isam2->mhCalculateBestEstimate(); //[6]
  
    cout << "== MH-CAL_BEST_EST ==" << endl;
    
    /*
    for (size_t k = 0; k < (i + 2); ++k) {
      vector<Pose3> out_arr = mh_results.mhAt<Pose3>(X(k));
      
      cout << out_arr.size() << endl;
      for (size_t h = 0; h < out_arr.size(); ++h) {

        cout << i << ": X(" << k << "): h: " << h << endl << out_arr[h] << endl;
      }
      cout << endl;
    }
    cout << "== MH-PRINT_RESULT ==" << endl;
    // */

    //================================ LOOP =========================
    if ((i+1)%loop_interval == 0) {
    //if (false) {
      
      //if ((i+1) == pose_num) continue;
      
      //==== LOOP ====
      loop_count++;
        
      const int loop_idx = ((i+1) - loop_interval)/2 + 2; 
      
      if (false) {
      //if (true) {
        // MH-LOOP

        std::vector<Pose3> loop_arr; //
      
        TT(0, 3) = 1.5*loop_interval; 
        loop_arr.push_back(Pose3(TT));
        
        TT(0, 3) = 2.5*loop_interval; 
        loop_arr.push_back(Pose3(TT));
     
        mh_isam2->createNextHypoLayer(loop_arr.size(), 6); //mhsiao: NOTICE: new variable must extend from the old one that is in the current last HypoLayer
      
        mh_graph->add(MHBetweenFactor<Pose3>(X(loop_idx), X(i+1), loop_arr, pose_noise_model)); //[4] MHBetweenFactor
        
        mh_isam2->assocLatestHypoLayerWith(mh_graph->back()); //mhiso: only do this when a new MH-NonlinearFactor is added

        //TODO: expand MHISAM2::theta_ at given keys later... set up the list here
        mh_isam2->setVariableToExpand(X(i+1), mh_isam2->getLastHypoLayer());

      } else {
        // SG-LOOP
        std::vector<Pose3> loop_arr; //
      
        TT(0, 3) = 1.5*(i + 1 - loop_idx); 
        loop_arr.push_back(Pose3(TT));
     
        mh_graph->add(MHBetweenFactor<Pose3>(X(loop_idx), X(i+1), loop_arr, pose_noise_model)); //[4] MHBetweenFactor

      }
    
      mh_isam2->update(*mh_graph); //[5]
      mh_graph->resize(0);
      mh_results = mh_isam2->mhCalculateBestEstimate(); //[6]
      
      /* 
      //TODO: refine result
      for (int lp = 0; lp < 10; ++lp) {
        mh_isam2->update(); //[5]
        mh_results = mh_isam2->mhCalculateBestEstimate(); //[6]
      }
      // */
     
      /* 
      for (size_t k = 0; k < (i + 2); ++k) {
        vector<Pose3> out_arr = mh_results.mhAt<Pose3>(X(k));
      
        cout << out_arr.size() << endl;
        for (size_t h = 0; h < out_arr.size(); ++h) {

          cout << i << ": loop_X(" << k << "): h: " << h << endl << out_arr[h] << endl;
        }
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

  //*
  // Output each MHGV
  ofstream outfile;
  string file_name = "MH_ISAM2_TEST_01";
  outfile.open(file_name + ".txt");
  outfile << "total_time:  " << total_time << endl;
  outfile << "loop_count:  " << loop_count << endl;
  for (size_t i = 0; i < (pose_num + 1); ++i) {
    vector<Pose3> out_arr = mh_results.mhAt<Pose3>(X(i));
    outfile << "out_X(" << i << "): " << out_arr.size() << endl;
    for (size_t h = 0; h < out_arr.size(); ++h) {
      Vector tt = out_arr[h].translation().vector();
      outfile << tt(0) << " " << tt(1) << " " << tt(2) << endl;
    }
    outfile << endl;
  }
  outfile.close();
  cout << "output " << file_name << ".txt file." << endl;
  // */

  //*
  // Output each max_hypo
  ofstream outfile_hypo;
  string hypo_file_name = "MH_ISAM2_TEST_01_hypo";
  outfile_hypo.open(hypo_file_name + ".txt");
  outfile_hypo << "total_time:  " << total_time << endl;
  outfile_hypo << "loop_count:  " << loop_count << endl;
  
  int hypo_count = 0;
  MHISAM2::HypoList& max_hypo_list = mh_isam2->getLastHypoLayer()->getNodeList();
  for (MHISAM2::HypoListIter it = max_hypo_list.begin(); it != max_hypo_list.end(); ++it) {
    outfile_hypo << "==== hypo: " << hypo_count << " ====" << endl;
    for (size_t i = 0; i < (pose_num + 1); ++i) {
      Pose3 out_pose = mh_results.mhAtHypo<Pose3>(X(i), (*it));
      outfile_hypo << "out_X(" << i << "): " << endl;
     
      Vector tt = out_pose.translation().vector();
      outfile_hypo << tt(0) << " " << tt(1) << " " << tt(2) << endl;
      outfile_hypo << endl;
    }
    outfile_hypo << endl;
    hypo_count++;
  }
  outfile_hypo.close();
  cout << "output " << hypo_file_name << ".txt file." << endl;
  // */

  string graph_file_name = "MH_ISAM2_TEST_01_graph.dot";
  mh_isam2->saveGraph(graph_file_name, gtsam::SingleTypeKeyFormatter);
  cout << "output " << graph_file_name << " file." << endl;
  

  return 0;
}


