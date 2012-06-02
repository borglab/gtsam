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
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>

// Use pose2SLAM namespace for specific SLAM instance
namespace pose2SLAM {

  using namespace gtsam;

  /// Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
  struct Values: public gtsam::Values {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
        gtsam::Values(values) {
    }

    // Convenience for MATLAB wrapper, which does not allow for identically named methods

    /// get a pose
    Pose2 pose(Key i) const { return at<Pose2>(i); }

    /// insert a pose
    void insertPose(Key i, const Pose2& pose) { insert(i, pose); }
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

    /// Adds a Pose2 prior with mean p and given noise model on pose i
    void addPrior(Key i, const Pose2& p, const SharedNoiseModel& model);

    /// Creates a hard constraint for key i with the given Pose2 p.
    void addPoseConstraint(Key i, const Pose2& p);

    /// Creates an odometry factor between poses with keys i1 and i2
    void addOdometry(Key i1, Key i2, const Pose2& z, const SharedNoiseModel& model);

    /// AddConstraint adds a soft constraint between factor between keys i and j
    void addConstraint(Key i1, Key i2, const Pose2& z, const SharedNoiseModel& model) {
    	addOdometry(i1,i2,z,model); // same for now
    }

    /// Optimize
    Values optimize(const Values& initialEstimate) const;

    /// Return a Marginals object
    Marginals marginals(const Values& solution) const {
    	return Marginals(*this,solution);
    }

  };

} // pose2SLAM



