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

#pragma once

namespace gtsam{

/**
 * special VectorConfig derived class that knows we are dealing with Pose3 objects
 * should be more elegant in later version of gtsam
 */
class DCVectorConfig : public gtsam::VectorConfig {
public:
  gtsam::VectorConfig operator+(const gtsam::VectorConfig & delta) const;
  void operator+=(const gtsam::VectorConfig & delta);
};

/**
 * Config that knows about points and poses
 */
class VSLAMConfig {

 private:
  typedef std::map<int, gtsam::Pose3> PoseMap;
  typedef std::map<int, gtsam::Point3> PointMap;
  PointMap landmarkPoints;
  PoseMap cameraPoses;

 public:
  typedef std::map<std::string, Vector>::const_iterator const_iterator;
  /**
   * default constructor
   */
  VSLAMConfig() {}

  /**
   * constructor that loads from file
   */
  VSLAMConfig(std::string& path, int num_of_frames) { load(path, num_of_frames);}

  /**
   * constructor from an VectorConfig
   */
  VSLAMConfig(gtsam::VectorConfig & Vectorconfig);

  /*
   * copy constructor
   */
  VSLAMConfig(const VSLAMConfig& original):
  	cameraPoses(original.cameraPoses), landmarkPoints(original.landmarkPoints){}

	/**
	 * Exponential map: takes 6D vectors in VectorConfig
	 * and applies them to the poses in the VSLAMConfig.
	 * Needed for use in nonlinear optimization
	 */
  VSLAMConfig exmap(const gtsam::VectorConfig & delta) const;

  /**
   * get the VectorConfig, poses have names x1,x2, and landmarks l1,l2,...
   */
  DCVectorConfig getVectorConfig() const;



  /**
   *  load values from files.
   */
  void load(std::string& path, int num_of_frames);

  /**
   * flush the poses into files (results),
   */
  void flush(int referenceMarker, const std::string& path);

  PoseMap::const_iterator cameraIteratorBegin() const  { return cameraPoses.begin();}
  PoseMap::const_iterator cameraIteratorEnd() const   { return cameraPoses.end();}
  PointMap::const_iterator landmarkIteratorBegin() const { return landmarkPoints.begin();}
  PointMap::const_iterator landmarkIteratorEnd() const  { return landmarkPoints.end();}

  /**
   * print
   */
  void print(const std::string& s = "") const;

  /**
   * Retrieve robot pose
   */
  bool cameraPoseExists(int i) const
  {
    PoseMap::const_iterator it = cameraPoses.find(i);
    if (it==cameraPoses.end())
      return false;
    return true;
  }

  gtsam::Pose3 cameraPose(int i) const {
    PoseMap::const_iterator it = cameraPoses.find(i);
    if (it==cameraPoses.end())
      throw(std::invalid_argument("robotPose: invalid key"));
    return it->second;
  }

  /**
   * Check whether a landmark point exists
   */
  bool landmarkPointExists(int i) const
  {
    PointMap::const_iterator it = landmarkPoints.find(i);
    if (it==landmarkPoints.end())
      return false;
    return true;
  }

  /**
   * Retrieve landmark point
   */
  gtsam::Point3 landmarkPoint(int i) const {
    PointMap::const_iterator it = landmarkPoints.find(i);
    if (it==landmarkPoints.end())
      throw(std::invalid_argument("markerPose: invalid key"));
    return it->second;
  }

  /**
   * check whether two configs are equal
   */
  bool equals(const VSLAMConfig& c) ;
  void addCameraPose(const int i, gtsam::Pose3 cp);
  void addLandmarkPoint(const int i, gtsam::Point3 lp);

  void removeCameraPose(const int i);
  void removeLandmarkPose(const int i);

  void clear() {landmarkPoints.clear(); cameraPoses.clear();}

  inline size_t size(){
    return landmarkPoints.size() + cameraPoses.size();
  }
};

} // namespace gtsam

