/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  planarSLAM.h
 *  @brief: bearing/range measurements in 2D plane
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/geometry/Pose2.h>

// Use planarSLAM namespace for specific SLAM instance
namespace planarSLAM {

  using namespace gtsam;

  /// Convenience function for constructing a pose key
  inline Symbol PoseKey(Index j) { return Symbol('x', j); }

  /// Convenience function for constructing a pose key
  inline Symbol PointKey(Index j) { return Symbol('l', j); }

  /*
   * List of typedefs for factors
   */
  /// A hard constraint for PoseKeys to enforce particular values
  typedef NonlinearEquality<Pose2> Constraint;
  /// A prior factor to bias the value of a PoseKey
  typedef PriorFactor<Pose2> Prior;
  /// A factor between two PoseKeys set with a Pose2
  typedef BetweenFactor<Pose2> Odometry;
  /// A factor between a PoseKey and a PointKey to express difference in rotation (set with a Rot2)
  typedef BearingFactor<Pose2, Point2> Bearing;
  /// A factor between a PoseKey and a PointKey to express distance between them (set with a double)
  typedef RangeFactor<Pose2, Point2> Range;
  /// A factor between a PoseKey and a PointKey to express difference in rotation and location
  typedef BearingRangeFactor<Pose2, Point2> BearingRange;

  /**  Values class, using specific PoseKeys and PointKeys
   * Mainly as a convenience for MATLAB wrapper, which does not allow for identically named methods
   */
  struct Values: public gtsam::Values {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
      gtsam::Values(values) {
    }

    /// get a pose
    Pose2 pose(Index key) const { return at<Pose2>(PoseKey(key)); }

    /// get a point
    Point2 point(Index key) const { return at<Point2>(PointKey(key)); }

    /// insert a pose
    void insertPose(Index key, const Pose2& pose) { insert(PoseKey(key), pose); }

    /// insert a point
    void insertPoint(Index key, const Point2& point) { insert(PointKey(key), point); }
  };

  /// Creates a NonlinearFactorGraph with the Values type
  struct Graph: public NonlinearFactorGraph {

    /// Default constructor for a NonlinearFactorGraph
    Graph(){}

    /// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph);

    /// Biases the value of PoseKey key with Pose2 p given a noise model
    void addPrior(Index poseKey, const Pose2& pose, const SharedNoiseModel& noiseModel);

    /// Creates a hard constraint to enforce Pose2 p for PoseKey poseKey's value
    void addPoseConstraint(Index poseKey, const Pose2& pose);

    /// Creates a factor with a Pose2 between PoseKeys poseKey and pointKey (poseKey.e. an odometry measurement)
    void addOdometry(Index poseKey1, Index poseKey2, const Pose2& odometry, const SharedNoiseModel& model);

    /// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in rotation
    void addBearing(Index poseKey, Index pointKey, const Rot2& bearing, const SharedNoiseModel& model);

    /// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in location
    void addRange(Index poseKey, Index pointKey, double range, const SharedNoiseModel& model);

    /// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in rotation and location
    void addBearingRange(Index poseKey, Index pointKey, const Rot2& bearing, double range, const SharedNoiseModel& model);

    /// Optimize
    Values optimize(const Values& initialEstimate) {
      typedef NonlinearOptimizer<Graph> Optimizer;
      return *Optimizer::optimizeLM(*this, initialEstimate,
          NonlinearOptimizationParameters::LAMBDA);
    }
  };

  /// Optimizer
  typedef NonlinearOptimizer<Graph> Optimizer;

} // planarSLAM


