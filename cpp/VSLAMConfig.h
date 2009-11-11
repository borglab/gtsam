/**
 * @file   VSLAMConfig.h
 * @brief  Config for VSLAM
 * @author Alireza Fathi
 * @author Carlos Nieto
 */

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include "VectorConfig.h"
#include "Pose3.h"
#include "Cal3_S2.h"
#include "Testable.h"

#pragma once

namespace gtsam{

/**
 * Config that knows about points and poses
 */
class VSLAMConfig : Testable<VSLAMConfig> {

 private:
  typedef std::map<int, Pose3> PoseMap;
  typedef std::map<int, Point3> PointMap;
  PointMap landmarkPoints_;
  PoseMap cameraPoses_;

 public:
  typedef std::map<std::string, Vector>::const_iterator const_iterator;
  /**
   * default constructor
   */
  VSLAMConfig() {}

  /*
   * copy constructor
   */
  VSLAMConfig(const VSLAMConfig& original):
  	cameraPoses_(original.cameraPoses_), landmarkPoints_(original.landmarkPoints_){}

	/**
	 * Exponential map: takes 6D vectors in VectorConfig
	 * and applies them to the poses in the VSLAMConfig.
	 * Needed for use in nonlinear optimization
	 */
  VSLAMConfig exmap(const VectorConfig & delta) const;

  PoseMap::const_iterator cameraIteratorBegin() const  { return cameraPoses_.begin();}
  PoseMap::const_iterator cameraIteratorEnd() const   { return cameraPoses_.end();}
  PointMap::const_iterator landmarkIteratorBegin() const { return landmarkPoints_.begin();}
  PointMap::const_iterator landmarkIteratorEnd() const  { return landmarkPoints_.end();}

  /**
   * print
   */
  void print(const std::string& s = "") const;

  /**
   * Retrieve robot pose
   */
  bool cameraPoseExists(int i) const
  {
    PoseMap::const_iterator it = cameraPoses_.find(i);
    if (it==cameraPoses_.end())
      return false;
    return true;
  }

  Pose3 cameraPose(int i) const {
    PoseMap::const_iterator it = cameraPoses_.find(i);
    if (it==cameraPoses_.end())
      throw(std::invalid_argument("robotPose: invalid key"));
    return it->second;
  }

  /**
   * Check whether a landmark point exists
   */
  bool landmarkPointExists(int i) const
  {
    PointMap::const_iterator it = landmarkPoints_.find(i);
    if (it==landmarkPoints_.end())
      return false;
    return true;
  }

  /**
   * Retrieve landmark point
   */
  Point3 landmarkPoint(int i) const {
    PointMap::const_iterator it = landmarkPoints_.find(i);
    if (it==landmarkPoints_.end())
      throw(std::invalid_argument("markerPose: invalid key"));
    return it->second;
  }

  /**
   * check whether two configs are equal
   */
  bool equals(const VSLAMConfig& c, double tol=1e-6) const;
  void addCameraPose(const int i, Pose3 cp);
  void addLandmarkPoint(const int i, Point3 lp);

  void removeCameraPose(const int i);
  void removeLandmarkPose(const int i);

  void clear() {landmarkPoints_.clear(); cameraPoses_.clear();}

  inline size_t size(){
    return landmarkPoints_.size() + cameraPoses_.size();
  }
};

} // namespace gtsam

