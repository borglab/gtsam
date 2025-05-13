/**
 * @file    easySLAMGraph.h
 * @brief   A factor graph for the EasySLAM problem
 * @author  Alireza Fathi
 */

#pragma once

#include <gtsam/NonlinearFactorGraph.h>

#include <map>
#include <set>
#include <vector>

#include "CameraMarkerFactor.h"
#include "CameraMarkerFactor0.h"
#include "CameraMarkerFactor1.h"
#include "EasySLAMConfig.h"

class EasySLAMGraph : public gtsam::NonlinearFactorGraph {
 private:
  typedef boost::shared_ptr<CameraMarkerFactor> shared_ptr;
  typedef boost::shared_ptr<CameraMarkerFactor0> shared_ptr0;
  typedef boost::shared_ptr<CameraMarkerFactor1> shared_ptr1;

 public:
  /**
   * Default Constructor
   */
  EasySLAMGraph() {}

  ~EasySLAMGraph() {}

  void load(int refMarker, std::vector<ARRobotMarker*> markers,
            gtsam::Cal3_S2 K, double markerSize);

  /**
   * Constructor that loads from files
   * @param path the directory in which files reside
   * @param nrFrames the number of frames to load
   * @return new factor graph
   */
  EasySLAMGraph(const std::string& path, int nrFrames);

  /**
   * Loads a particular frame with optional reference marker and adds it to the
   * end of EasySLAMGraph
   * @param path the directory in which files reside
   * @param frameNumber that is going to be loaded
   * @param refMarker optional reference marker
   * CHRIS MADE THIS
   */
  void loadAFrame(const std::string& path, int frameNumber, int refMarker = -1);

  /**
   * Loads a particular frame and adds it to the end of EasySLAMGraph
   * @param refMarker set the reference marker
   * @param path the directory in which files reside
   * @param frameNumber that is going to be loaded
   */
  void loadAFrame(int refMarker, const std::string& path, int frameNumber);

  /**
   * Takes a Marker und pushed a factor into the NLF graph
   * @param markers
   */
  void insertMarker(std::vector<ARRobotMarker*> markers, int frameNumber,
                    int refMarker = -1);

  /**
   * print out graph
   */
  void print(const std::string& s = "") const {
    NonlinearFactorGraph::print(s);
  }

  void dump(const std::string& path);
  void load_dumped(const std::string& path);
};
