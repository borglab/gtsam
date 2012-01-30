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

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

// Use pose2SLAM namespace for specific SLAM instance
namespace pose2SLAM {

  using namespace gtsam;

  /// Keys with Pose2 and symbol 'x'
  typedef TypedSymbol<Pose2, 'x'> PoseKey;

  /// Values class, inherited from Values, using PoseKeys
  struct Values: public gtsam::Values<PoseKey> {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values<PoseKey>& values) :
        gtsam::Values<PoseKey>(values) {
    }

    // Convenience for MATLAB wrapper, which does not allow for identically named methods

    /// get a pose
    Pose2 pose(int key) const { return (*this)[PoseKey(key)]; }

    /// insert a pose
    void insertPose(int key, const Pose2& pose) { insert(PoseKey(key), pose); }
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
  typedef NonlinearEquality<Values, PoseKey> HardConstraint;
  /// A prior factor on a pose with Pose2 data type.
  typedef PriorFactor<Values, PoseKey> Prior;
  /// A factor to add an odometry measurement between two poses.
  typedef BetweenFactor<Values, PoseKey> Odometry;

  /// Graph
  struct Graph: public NonlinearFactorGraph<Values> {

    /// Default constructor for a NonlinearFactorGraph
    Graph(){}

    /// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
    Graph(const NonlinearFactorGraph<Values>& graph);

    /// Adds a Pose2 prior with a noise model to one of the keys in the nonlinear factor graph
    void addPrior(const PoseKey& i, const Pose2& p, const SharedNoiseModel& model);

    /// Creates a hard constraint for key i with the given Pose2 p.
    void addPoseConstraint(const PoseKey& i, const Pose2& p);

    /// Creates a between factor between keys i and j with a noise model with Pose2 z in the graph
    void addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
        const SharedNoiseModel& model);

    /// Optimize
    Values optimize(const Values& initialEstimate) {
      typedef NonlinearOptimizer<Graph, Values> Optimizer;
      return *Optimizer::optimizeLM(*this, initialEstimate,
                  NonlinearOptimizationParameters::LAMBDA);
    }
  };

  /// The sequential optimizer
  typedef NonlinearOptimizer<Graph, Values, GaussianFactorGraph,
      GaussianSequentialSolver> OptimizerSequential;

  /// The multifrontal optimizer
  typedef NonlinearOptimizer<Graph, Values, GaussianFactorGraph,
      GaussianMultifrontalSolver> Optimizer;

} // pose2SLAM



