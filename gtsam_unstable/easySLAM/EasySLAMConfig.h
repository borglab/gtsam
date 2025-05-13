/**
 * @file   EasySLAMConfig.h
 * @brief  Solution of EasySLAM
 * @author Alireza Fathi
 * @author Frank Dellaert
 */

#include <gtsam/Cal3_S2.h>
#include <gtsam/FGConfig.h>
#include <gtsam/Pose3.h>

#include <map>
#include <string>
#include <vector>

#include "ARRobotMarker.h"
#include "EasyVectorConfig.h"

#pragma once

class EasySLAMConfig {
 private:
  typedef std::map<int, gtsam::Pose3> poseMap;
  typedef poseMap::iterator iterator;
  typedef poseMap::const_iterator const_iterator;
  poseMap markerPoses;
  poseMap robotPoses;

 public:
  /**
   * default constructor
   */
  EasySLAMConfig() {}

  /**
   * constructor that loads from file
   */
  EasySLAMConfig(std::string& path, int num_of_frames) {
    load(path, num_of_frames);
  }

  EasySLAMConfig(gtsam::FGConfig& fgconfig);

  /**
   * get the FGConfig _fgconfig
   */
  EasyVectorConfig getFGConfig() const;

  /**
   *  load from marker locations, calibration and markerSize
   */
  void load(int refMarker, std::vector<ARRobotMarker*> markers,
            gtsam::Cal3_S2 K, double markerSize);

  /**
   *  load from files
   */
  void load(std::string& path, int num_of_frames);

  /**
   *  load one frane from files
   * @param path the path
   * @param frameNumber the frame which should be load
   * @param refMarker the optional reference marker
   */
  void loadAFrame(std::string& path, int frameNumber, int refMarker = -1);

  /**
   *  load one frane from files
   */
  void loadAFrame(int refMarker, std::string& path, int frameNumber);

  /**
   * flush the poses into files (results)
   */
  void flush(int referenceMarker, const std::string& path);

  /**
   * dump the EasySLAMConfig into a file, which later can be used to load from
   * it
   */
  void dump(const std::string& path);

  /**
   * Loads the EasySLAMConfig from a file that has been dumped.
   */
  void load_dumped(const std::string& path);

  iterator robotIteratorBegin() { return robotPoses.begin(); }
  iterator robotIteratorEnd() { return robotPoses.end(); }
  iterator markerIteratorBegin() { return markerPoses.begin(); }
  iterator markerIteratorEnd() { return markerPoses.end(); }

  /**
   * print
   */
  void print(const std::string& s = "");

  /**
   * Retrieve robot pose
   */
  bool robotPoseExists(int i) const {
    const_iterator it = robotPoses.find(i);
    if (it == robotPoses.end()) return false;
    return true;
  }

  gtsam::Pose3 robotPose(int i) const {
    const_iterator it = robotPoses.find(i);
    if (it == robotPoses.end())
      throw(std::invalid_argument("robotPose: invalid key"));
    return it->second;
  }

  /**
   * Retrieve marker pose
   */
  bool markerPoseExists(int i) const {
    const_iterator it = markerPoses.find(i);
    if (it == markerPoses.end()) return false;
    return true;
  }

  gtsam::Pose3 markerPose(int i) const {
    const_iterator it = markerPoses.find(i);
    if (it == markerPoses.end())
      throw(std::invalid_argument("markerPose: invalid key"));
    return it->second;
  }

  /**
   * transformations of full configurations
   * follows convention of transform_to/from methods in gtsam geometry
   */
  EasySLAMConfig transform_to(const gtsam::Pose3& pose);

  /**
   * check whether two configs are equal
   */
  bool equals(EasySLAMConfig& c, double tol = 1e-9);

  /**
   * add/remove methods
   */
  void addRobotPose(const int i, gtsam::Pose3 rp);
  void addMarkerPose(const int i, gtsam::Pose3 mp);

  void removeRobotPose(const int i);
  void removeMarkerPose(const int i);

  void clear() {
    markerPoses.clear();
    robotPoses.clear();
  }

  inline size_t size() { return markerPoses.size() + robotPoses.size(); }

  friend bool assert_equal(const EasySLAMConfig& a, const EasySLAMConfig& b,
                           double tol);
};

bool assert_equal(const EasySLAMConfig& a, const EasySLAMConfig& b,
                  double tol = 1e-9);
