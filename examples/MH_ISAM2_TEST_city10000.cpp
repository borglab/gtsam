#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <vector>
#include <fstream>
#include <string>
#include <time.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::X;

// Testing params
const size_t max_loop_count = 8000; //200 //2000 //8000

const bool active_all_loop_detach = false;
//const bool active_all_loop_detach = true;

noiseModel::Diagonal::shared_ptr prior_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0001, 0.0001, 0.0001).finished());

noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 1.0/50.0, 1.0/50.0, 1.0/100.0).finished());

/* ************************************************************************* */

int main(int argc, char* argv[]) {
  
  //ifstream in("../data/mh_T1_city10000_04.txt"); //Type #1 only
  //ifstream in("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  ifstream in("../data/mh_T1_T3_city10000_04.txt"); //Type #1 + Type #3

  size_t pose_count = 0;
  size_t loop_count = 0;

  std::list<double> time_list;

  ISAM2Params parameters; 
  MHParams mh_parameters(10, 30); //T1: (10, 30), T3: (10, 10)

  mh_parameters.isPrintPruningDetails = true;
    
  //parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1);  //_initialDelta = 1.0 //Dogleg NOT implemented yet!!!!
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0); //_wildfireThreshold = 0.001
  
  parameters.relinearizeThreshold = 0.01;
  
  parameters.relinearizeSkip = 1;
  MHISAM2* mh_isam2 = new MHISAM2(parameters, mh_parameters);

  NonlinearFactorGraph* mh_graph = new NonlinearFactorGraph(); 

  Values mh_init_values; 
  Values mh_results; 
 
  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;
  
  std::vector<Pose2> prior_arr;
  prior_arr.push_back(Pose2(x, y, rad));
 
  MHGenericValue<Pose2> prior_value(prior_arr[0], mh_isam2->getFirstHypoLayer());
  
  mh_init_values.mhInsert(X(0), prior_value);
  pose_count++;

  mh_graph->add(MHPriorFactor<Pose2>(X(0), prior_arr, prior_noise_model));

  mh_isam2->update(*mh_graph, mh_init_values);
  mh_graph->resize(0);
  mh_init_values.clear();
  mh_results = mh_isam2->mhCalculateBestEstimate();
  
  //*
  size_t key_s = 0;
  size_t key_t = 0;
  
  clock_t start_time = clock();
  
  string str;
  while (getline(in, str) && loop_count < max_loop_count) {
    
    //cout << str << endl; 
    vector<string> parts;
    split(parts, str, is_any_of(" "));
    
    key_s = stoi(parts[1]);
    key_t = stoi(parts[3]);
    int empty = stoi(parts[4]); // 0 or 1
    bool allow_empty = (empty == 0) ? false : true; // 0 or 1
    int m_num = stoi(parts[5]);
    vector<double> x_arr(m_num);
    vector<double> y_arr(m_num);
    vector<double> rad_arr(m_num);
    for (int i = 0; i < m_num; ++i) {
      x_arr[i] = stod(parts[6 + 3*i]);
      y_arr[i] = stod(parts[7 + 3*i]);
      rad_arr[i] = stod(parts[8 + 3*i]);
    }
    
    std::vector<Pose2> odom_arr;
    for (int i = 0; i < m_num; ++i) {
      odom_arr.push_back(Pose2(x_arr[i], y_arr[i], rad_arr[i]));
    }
    
    //flags: m_num, is_loop, is_detach (for loop only)
    const bool is_loop_detach = (active_all_loop_detach || allow_empty) && (key_s != key_t - 1); //active && loop

    if (key_s == key_t - 1) { //odom
      if (m_num > 1) { //MH
        mh_isam2->createNextHypoLayer(odom_arr.size(), 3); //mhsiao: should get dim from factor type
      }
    } else { //loop
      if (is_loop_detach) { //SG/MH w/ detach = MH
        mh_isam2->createNextHypoLayer(odom_arr.size(), 3, true, true); //mhsiao: should get dim from factor type
      } else if (m_num > 1) { //MH w/o detach
        mh_isam2->createNextHypoLayer(odom_arr.size(), 3, true, false); //mhsiao: should get dim from factor type
      } else { //SG
        mh_isam2->accumulateCommonDim(3);
      }
    }

    if (key_s == key_t - 1) { //new X(key)
      MHGenericValue<Pose2> pose_value(mh_results.mhAt<Pose2>(X(key_s)), odom_arr, mh_isam2->getLastHypoLayer() ); 
      mh_init_values.mhInsert(X(key_t), pose_value);
      pose_count++;
    } else { //loop
      if (m_num > 1 || is_loop_detach) {
        mh_isam2->setVariableToExpand(X(key_t), mh_isam2->getLastHypoLayer());
      }
      loop_count++;
    }

    if (is_loop_detach) { //loop w/ detach
      mh_graph->add(MHBetweenFactor<Pose2>(X(key_s), X(key_t), odom_arr, pose_noise_model, true));
    } else { //loop w/o detach || NOT loop
      mh_graph->add(MHBetweenFactor<Pose2>(X(key_s), X(key_t), odom_arr, pose_noise_model));
    }
    
    if (m_num > 1 || is_loop_detach) {
      mh_isam2->assocLatestHypoLayerWith(mh_graph->back());
    }

    mh_isam2->update(*mh_graph, mh_init_values);
    mh_graph->resize(0);
    mh_init_values.clear();
    mh_results = mh_isam2->mhCalculateBestEstimate();
  
    //* 
    if (loop_count%50 == 0 && key_s != key_t - 1) {
       std::cout << "loop_count: " << loop_count << std::endl;
       
       //MHISAM2::HypoList& curr_hypo_list = mh_isam2->getLastHypoLayer()->getNodeList();
       //std::cout << "Last layer hypos:  " << curr_hypo_list.size() << std::endl;
       
       std::cout << "acc_time:  " << time_list.back() << std::endl;
    }
    // */

    
    if (key_s == key_t - 1) {
      clock_t cur_time = clock();
      time_list.push_back(cur_time - start_time);
    }
    
    if (time_list.size()%100 == 0 && (key_s == key_t - 1)) {
      string step_file_idx = std::to_string(100000 + time_list.size());
      
      ofstream step_outfile;
      string step_file_name = "mh_step_files/MH_ISAM2_TEST_city10000_S" + step_file_idx;
      step_outfile.open(step_file_name + ".txt");
      
      MHISAM2::HypoListIter it = mh_isam2->getLastHypoLayer()->getNodeList().begin();
      for (size_t i = 0; i < (pose_count); ++i) {
        Pose2 out_pose = mh_results.mhAtHypo<Pose2>(X(i), (*it));
     
        Vector tt = out_pose.translation().vector();
        step_outfile << tt(0) << " " << tt(1) << " "; // [x1 y1] [x2 y2] ...
      }
      step_outfile.close();
    }

  } // END while loop

  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;
  cout << "# layers: " << mh_isam2->getLastHypoLayer()->getLayerIdx() << endl;
  
  mh_isam2->printAllFinalHypo();
  //* 
  
  ofstream outfile;
  string file_name = "MH_ISAM2_TEST_city10000";
  outfile.open(file_name + ".txt");
  
  for (size_t i = 0; i < (key_t + 1); ++i) {
    vector<Pose2> out_arr = mh_results.mhAt<Pose2>(X(i));
  
    for (size_t h = 0; h < out_arr.size(); ++h) {
    
      outfile << out_arr[h].x() << " " << out_arr[h].y() << " " << out_arr[h].theta() << endl; //WRONG FORMAT... JUST FOR SIMPLE SHOW
    }
  }
  outfile.close();
  cout << "output " << file_name << ".txt file." << endl;
  // */
  
  //*
  // Output each hypo
  ofstream outfile_hypo;
  string hypo_file_name = "MH_ISAM2_TEST_city10000_hypos";
  outfile_hypo.open(hypo_file_name + ".txt");
  
  int hypo_count = 0;
  MHISAM2::HypoList& max_hypo_list = mh_isam2->getLastHypoLayer()->getNodeList();
  for (MHISAM2::HypoListIter it = max_hypo_list.begin(); it != max_hypo_list.end(); ++it) {
    for (size_t i = 0; i < (pose_count); ++i) {
      Pose2 out_pose = mh_results.mhAtHypo<Pose2>(X(i), (*it));
     
      Vector tt = out_pose.translation().vector();
      outfile_hypo << tt(0) << " " << tt(1) << " "; // [x1 y1] [x2 y2] ...
    }
    outfile_hypo << endl;
    hypo_count++;
  }
  outfile_hypo.close();
  cout << "output " << hypo_file_name << ".txt file." << endl;
  // */
  
  //*
  // Output each mode
  ofstream outfile_mode;
  string mode_file_name = "MH_ISAM2_TEST_city10000_modes";
  outfile_mode.open(mode_file_name + ".txt");
  outfile_mode << "total_time:  " << total_time << endl;
  outfile_mode << "# layers: " << mh_isam2->getLastHypoLayer()->getLayerIdx() << endl;
  outfile_mode << "overall original hypos:  " << mh_isam2->calculateExpGrowHypoNum() << endl;
  outfile_mode << "overall final hypos:  " << max_hypo_list.size() << endl;
  outfile_mode << "loop_count:  " << loop_count << endl;
  
  hypo_count = 0;
  for (MHISAM2::HypoListIter it = max_hypo_list.begin(); it != max_hypo_list.end(); ++it) {
    outfile_mode << "==== hypo: " << hypo_count << " ====" << endl;
    for (size_t a = 0; a < ((*it)->ancestor_arr_.size() + 1); ++a) {
      outfile_mode << (*it)->findAncestor(a)->mode_id_ << endl;
    }
    hypo_count++;
  }
  outfile_mode.close();
  cout << "output " << mode_file_name << ".txt file." << endl;

  //*
  ofstream outfile_time;
  string time_file_name = "MH_ISAM2_TEST_city10000_time";
  outfile_time.open(time_file_name + ".txt");
  for (auto acc_time : time_list) {
    outfile_time << acc_time << endl; 
  }
  outfile_time.close();
  cout << "output " << time_file_name << ".txt file." << endl;
  // */

  return 0;
}
