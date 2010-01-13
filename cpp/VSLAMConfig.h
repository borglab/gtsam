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

#include "Pose3Config.h"

#pragma once

namespace gtsam{

/**
 * Config that knows about points and poses
 */
class VSLAMConfig : Testable<VSLAMConfig> {

public:

 typedef Symbol<Pose3,'x'> PoseKey;
 typedef Symbol<Point3,'l'> PointKey;

private:

  LieConfig<PoseKey,  Pose3>  poses_;
	LieConfig<PointKey, Point3> points_;

public:

  /**
   * default constructor
   */
  VSLAMConfig() {}

  /**
   * print
   */
  void print(const std::string& s = "") const;

  /**
   * check whether two configs are equal
   */
  bool equals(const VSLAMConfig& c, double tol=1e-6) const;

  /**
   * Get Poses or Points
   */
  inline const Pose3&  operator[](const PoseKey&  key) const {return poses_[key];}
  inline const Point3& operator[](const PointKey& key) const {return points_[key];}

  // (Awkwardly named) backwards compatibility:

  inline bool cameraPoseExists   (const PoseKey&  key) const {return  poses_.exists(key);}
  inline bool landmarkPointExists(const PointKey& key) const {return points_.exists(key);}

  inline Pose3  cameraPose   (const PoseKey&  key) const {return  poses_[key];}
  inline Point3 landmarkPoint(const PointKey& key) const {return points_[key];}

  inline size_t size() const {return points_.size() + poses_.size();}
  inline size_t dim() const {return gtsam::dim(points_) + gtsam::dim(poses_);}

  // Imperative functions:

  inline void addCameraPose(const PoseKey& key, Pose3 cp) {poses_.insert(key,cp);}
  inline void addLandmarkPoint(const PointKey& key, Point3 lp) {points_.insert(key,lp);}

  inline void removeCameraPose(const PoseKey& key) { poses_.erase(key);}
  inline void removeLandmarkPose(const PointKey& key) { points_.erase(key);}

  inline void clear() {points_.clear(); poses_.clear();}

  friend VSLAMConfig expmap(const VSLAMConfig& c, const VectorConfig & delta);
};


/**
 * Exponential map: takes 6D vectors in VectorConfig
 * and applies them to the poses in the VSLAMConfig.
 * Needed for use in nonlinear optimization
 */
VSLAMConfig expmap(const VSLAMConfig& c, const VectorConfig & delta);
} // namespace gtsam

