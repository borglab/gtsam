/**
 * @file    VSLAMGraph.h
 * @brief   A factor graph for the VSLAM problem
 * @author  Alireza Fathi
 * @author  Carlos Nieto
 */


#pragma once

#include <vector>
#include <map>
#include <set>
#include <fstream>

#include <gtsam/NonlinearFactorGraph.h>
#include <gtsam/FactorGraph-inl.h>
#include "VSLAMFactor.h"
#include "VSLAMFactor0.h"
#include "StereoFactor.h"
#include "VSLAMConfig.h"


using namespace std;

/**
 * Non-linear factor graph for visual SLAM
 */
class VSLAMGraph : public gtsam::NonlinearFactorGraph<VSLAMConfig>{
private:
	int nFrames;
	typedef map <int, int> feat_ids_type;
	feat_ids_type feat_ids;

public:

  /** default constructor is empty graph */
  VSLAMGraph() {}

  /**
   * Constructor that loads measurements from file
   * @param path to the file
   */
  VSLAMGraph(const std::string& path);

  /**
   * Constructor that loads from VO file (not tested)
   * @param path to the file
   * @param nrFrames the number of frames to load
   * @return new factor graph
   */
  VSLAMGraph(const std::string& path, int nrFrames, double sigma, const gtsam::Cal3_S2& K);

  /**
   * print out graph
   */
  void print(const std::string& s = "") const {
    gtsam::NonlinearFactorGraph<VSLAMConfig>::print(s);
  }

  void load_dumped(const std::string& path);

  int Get_nFrames(){return nFrames;};
  int Get_nFeat_ids(){return feat_ids.size();};
  feat_ids_type* Get_feat_ids_map(){return &feat_ids;};
};
