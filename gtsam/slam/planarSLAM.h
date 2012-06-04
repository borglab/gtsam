/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  planarSLAM.h
 *  @brief bearing/range measurements in 2D plane
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
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>

/**
 * @defgroup SLAM
 */

// Use planarSLAM namespace for specific SLAM instance
namespace planarSLAM {

  using namespace gtsam;

  /*
   * List of typedefs for factors
   */

  /// A hard constraint for poses to enforce particular values
  typedef NonlinearEquality<Pose2> Constraint;
  /// A prior factor to bias the value of a pose
  typedef PriorFactor<Pose2> Prior;
  /// A factor between two poses set with a Pose2
  typedef BetweenFactor<Pose2> Odometry;
  /// A factor between a pose and a point to express difference in rotation (set with a Rot2)
  typedef BearingFactor<Pose2, Point2> Bearing;
  /// A factor between a pose and a point to express distance between them (set with a double)
  typedef RangeFactor<Pose2, Point2> Range;
  /// A factor between a pose and a point to express difference in rotation and location
  typedef BearingRangeFactor<Pose2, Point2> BearingRange;

  /*
   * Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
   * @ingroup SLAM
   */
  struct Values: public gtsam::Values {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
      gtsam::Values(values) {
    }

    /// get a pose
    Pose2 pose(Key i) const { return at<Pose2>(i); }

    /// get a point
    Point2 point(Key j) const { return at<Point2>(j); }

    /// insert a pose
    void insertPose(Key i, const Pose2& pose) { insert(i, pose); }

    /// insert a point
    void insertPoint(Key j, const Point2& point) { insert(j, point); }
  };

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @ingroup SLAM
   */
  struct Graph: public NonlinearFactorGraph {

    /// Default constructor for a NonlinearFactorGraph
    Graph(){}

    /// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph);

    /// Biases the value of pose key with Pose2 p given a noise model
    void addPrior(Key i, const Pose2& pose, const SharedNoiseModel& noiseModel);

    /// Creates a hard constraint on the ith pose
    void addPoseConstraint(Key i, const Pose2& pose);

    /// Creates an odometry factor between poses with keys i1 and i2
    void addOdometry(Key i1, Key i2, const Pose2& odometry, const SharedNoiseModel& model);

    /// Creates a bearing measurement from pose i to point j
    void addBearing(Key i, Key j, const Rot2& bearing, const SharedNoiseModel& model);

    /// Creates a range measurement from pose i to point j
    void addRange(Key i, Key k, double range, const SharedNoiseModel& model);

    /// Creates a range/bearing measurement from pose i to point j
    void addBearingRange(Key i, Key j, const Rot2& bearing, double range, const SharedNoiseModel& model);

    /// Optimize
    Values optimize(const Values& initialEstimate) const;

    /// Return a Marginals object
    Marginals marginals(const Values& solution) const {
    	return Marginals(*this,solution);
    }
  };

} // planarSLAM


