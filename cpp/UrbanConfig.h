/**
 * @file   UrbanConfig.h
 * @brief  Config for Urban application
 * @author Frank Dellaert
 * @author Viorela Ila
 */

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include "VectorConfig.h"
#include "Pose3.h"
#include "Point2.h"
#include "Testable.h"

#pragma once

namespace gtsam{

/**
 * Config that knows about points and poses
 */
class UrbanConfig : Testable<UrbanConfig> {

 private:
  typedef std::map<int, Pose3> PoseMap;
  typedef std::map<int, Point2> PointMap;
  PointMap landmarkPoints_;
  PoseMap robotPoses_;

 public:
  typedef std::map<std::string, Vector>::const_iterator const_iterator;
  typedef PoseMap::const_iterator const_Pose_iterator;
  typedef PointMap::const_iterator const_Point_iterator;
  /**
   * default constructor
   */
  UrbanConfig() {}

  /*
   * copy constructor
   */
  UrbanConfig(const UrbanConfig& original):
  	robotPoses_(original.robotPoses_), landmarkPoints_(original.landmarkPoints_){}

	/**
	 * Exponential map: takes 6D vectors in VectorConfig
	 * and applies them to the poses in the UrbanConfig.
	 * Needed for use in nonlinear optimization
	 */
  UrbanConfig exmap(const VectorConfig & delta) const;

  PoseMap::const_iterator robotIteratorBegin() const  { return robotPoses_.begin();}
  PoseMap::const_iterator robotIteratorEnd() const   { return robotPoses_.end();}
  PointMap::const_iterator landmarkIteratorBegin() const { return landmarkPoints_.begin();}
  PointMap::const_iterator landmarkIteratorEnd() const  { return landmarkPoints_.end();}

  /**
   * print
   */
  void print(const std::string& s = "") const;

  /**
   * Retrieve robot pose
   */
  bool robotPoseExists(int i) const
  {
    PoseMap::const_iterator it = robotPoses_.find(i);
    if (it==robotPoses_.end())
      return false;
    return true;
  }

  Pose3 robotPose(int i) const {
    PoseMap::const_iterator it = robotPoses_.find(i);
    if (it==robotPoses_.end())
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
  Point2 landmarkPoint(int i) const {
    PointMap::const_iterator it = landmarkPoints_.find(i);
    if (it==landmarkPoints_.end())
      throw(std::invalid_argument("markerPose: invalid key"));
    return it->second;
  }

  /**
   * check whether two configs are equal
   */
  bool equals(const UrbanConfig& c, double tol=1e-6) const;
  void addRobotPose(const int i, Pose3 cp);
  void addLandmark(const int i, Point2 lp);

  void removeRobotPose(const int i);
  void removeLandmark(const int i);

  void clear() {landmarkPoints_.clear(); robotPoses_.clear();}

  inline size_t size(){
    return landmarkPoints_.size() + robotPoses_.size();
  }
};

} // namespace gtsam

