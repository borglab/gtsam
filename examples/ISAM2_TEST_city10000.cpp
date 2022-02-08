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

//const bool is_with_ambiguity = false; //run original iSAM2 without ambiguities
const bool is_with_ambiguity = true; //run original iSAM2 with ambiguities

noiseModel::Diagonal::shared_ptr prior_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0001, 0.0001, 0.0001).finished());

noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 1.0/50.0, 1.0/50.0, 1.0/100.0).finished());

/* ************************************************************************* */

int main(int argc, char* argv[]) {
  
  //ifstream in("../data/mh_T1_city10000_04.txt"); //Type #1 only
  //ifstream in("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  ifstream in("../data/mh_T1_T3_city10000_04.txt"); //Type #1 + Type #3

  //ifstream in("../data/mh_All_city10000_groundtruth.txt");
  
  size_t pose_count = 0;
  size_t loop_count = 0;

  std::list<double> time_list;

  ISAM2Params parameters; 
  
  //parameters.optimizationParams = gtsam::ISAM2DoglegParams(0.1);  //_initialDelta = 1.0 //Dogleg NOT implemented yet!!!!
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0); //_wildfireThreshold = 0.001
  
  parameters.relinearizeThreshold = 0.01;
  
  parameters.relinearizeSkip = 1;
  ISAM2* isam2 = new ISAM2(parameters); //[2] MHISAM2

  NonlinearFactorGraph* graph = new NonlinearFactorGraph(); 

  Values init_values; 
  Values results; 
 
  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;

  Pose2 prior_pose(x, y, rad);
  
  init_values.insert(X(0), prior_pose);
  pose_count++;

  graph->add(PriorFactor<Pose2>(X(0), prior_pose, prior_noise_model));

  isam2->update(*graph, init_values);
  graph->resize(0);
  init_values.clear();
  results = isam2->calculateBestEstimate();
  
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
    int m_num = stoi(parts[5]);
    vector<double> x_arr(m_num);
    vector<double> y_arr(m_num);
    vector<double> rad_arr(m_num);
    for (int i = 0; i < m_num; ++i) {
      x_arr[i] = stod(parts[6 + 3*i]);
      y_arr[i] = stod(parts[7 + 3*i]);
      rad_arr[i] = stod(parts[8 + 3*i]);
    }

    Pose2 odom_pose;
    if (is_with_ambiguity) {
        // Get wrong intentionally
        int id = loop_count%m_num;
        odom_pose = Pose2(x_arr[id], y_arr[id], rad_arr[id]);
    } else {
        odom_pose = Pose2(x_arr[0], y_arr[0], rad_arr[0]);
    }
    
    if (key_s == key_t - 1) { //new X(key)
      init_values.insert(X(key_t), results.at<Pose2>(X(key_s))*odom_pose);
      pose_count++;
    } else { //loop
      loop_count++;
    }
    graph->add(BetweenFactor<Pose2>(X(key_s), X(key_t), odom_pose, pose_noise_model));

    isam2->update(*graph, init_values);
    graph->resize(0);
    init_values.clear();
    results = isam2->calculateBestEstimate();
    
    //*
    if (loop_count%50 == 0 && key_s != key_t - 1) {
       std::cout << "loop_count: " << loop_count << std::endl;
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
      string step_file_name = "step_files/ISAM2_TEST_city10000_S" + step_file_idx;
      step_outfile.open(step_file_name + ".txt");
      for (size_t i = 0; i < (key_t + 1); ++i) {
        Pose2 out_pose = results.at<Pose2>(X(i));
        step_outfile << out_pose.x() << " " << out_pose.y() << " " << out_pose.theta() << endl;
      }
      step_outfile.close();
    }
    
  }
  
  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;
  //* 
  
  ofstream outfile;
  string file_name = "ISAM2_TEST_city10000";
  outfile.open(file_name + ".txt");
  
  for (size_t i = 0; i < (key_t + 1); ++i) {
    Pose2 out_pose = results.at<Pose2>(X(i));
    
    outfile << out_pose.x() << " " << out_pose.y() << " " << out_pose.theta() << endl;
  }
  outfile.close();
  cout << "output " << file_name << ".txt file." << endl;
  // */
  
  //*
  ofstream outfile_time;
  string time_file_name = "ISAM2_TEST_city10000_time";
  outfile_time.open(time_file_name + ".txt");
  for (auto acc_time : time_list) {
    outfile_time << acc_time << endl; //WRONG FORMAT... JUST FOR SIMPLE SHOW
  }
  outfile_time.close();
  cout << "output " << time_file_name << ".txt file." << endl;
  // */
  return 0;
}
