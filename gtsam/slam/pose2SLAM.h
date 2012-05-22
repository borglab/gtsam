/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose2SLAM.h
 *  @brief: 2D Pose SLAM
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>

// Use pose2SLAM namespace for specific SLAM instance
namespace pose2SLAM {

  using namespace gtsam;

  /// Convenience function for constructing a pose key
  inline Symbol PoseKey(Index j) { return Symbol('x', j); }

  /// Values class, inherited from Values, using PoseKeys, mainly used as a convenience for MATLAB wrapper
  struct Values: public gtsam::Values {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
        gtsam::Values(values) {
    }

    // Convenience for MATLAB wrapper, which does not allow for identically named methods

    /// get a pose
    Pose2 pose(Index key) const { return at<Pose2>(PoseKey(key)); }

    /// insert a pose
    void insertPose(Index key, const Pose2& pose) { insert(PoseKey(key), pose); }
  };

  /**
   * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
   * @param n number of poses
   * @param R radius of circle
   * @param c character to use for keys
   * @return circle of n 2D poses
   */
  Values circle(size_t n, double R);

  /**
   * List of typedefs for factors
   */

  /// A hard constraint to enforce a specific value for a pose
  typedef NonlinearEquality<Pose2> HardConstraint;
  /// A prior factor on a pose with Pose2 data type.
  typedef PriorFactor<Pose2> Prior;
  /// A factor to add an odometry measurement between two poses.
  typedef BetweenFactor<Pose2> Odometry;

  /// Graph
  struct Graph: public NonlinearFactorGraph {

    /// Default constructor for a NonlinearFactorGraph
    Graph(){}

    /// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph);

    /// Adds a Pose2 prior with a noise model to one of the keys in the nonlinear factor graph
    void addPrior(Index i, const Pose2& p, const SharedNoiseModel& model);

    /// Creates a hard constraint for key i with the given Pose2 p.
    void addPoseConstraint(Index i, const Pose2& p);

    /// Creates a between factor between keys i and j with a noise model with Pose2 z in the graph
    void addOdometry(Index i, Index j, const Pose2& z,
        const SharedNoiseModel& model);

    /// AddConstraint adds a soft constraint between factor between keys i and j
    void addConstraint(Index i, Index j, const Pose2& z,
        const SharedNoiseModel& model) {
    	addOdometry(i,j,z,model); // same for now
    }

    /// Optimize
    Values optimize(const Values& initialEstimate) const;
  };

} // pose2SLAM



