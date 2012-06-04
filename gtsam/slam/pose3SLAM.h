/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose3SLAM.h
 *  @brief: 3D Pose SLAM
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>

/// Use pose3SLAM namespace for specific SLAM instance
namespace pose3SLAM {

  using namespace gtsam;

  /*
   * Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
   * @ingroup SLAM
   */
  struct Values: public gtsam::Values {

	  typedef boost::shared_ptr<Values> shared_ptr;

	  /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
        gtsam::Values(values) {
    }

    /**
     * Create a circle of n 3D poses tangent to circle of radius R, first pose at (R,0)
     * @param n number of poses
     * @param R radius of circle
     * @return circle of n 3D poses
     */
    static Values Circle(size_t n, double R);

    /// insert a pose
    void insertPose(Key i, const Pose3& pose) { insert(i, pose); }

    /// get a pose
    Pose3 pose(Key i) const { return at<Pose3>(i); }

    Vector xs() const; ///< get all x coordinates in a matrix
    Vector ys() const; ///< get all y coordinates in a matrix
    Vector zs() const; ///< get all z coordinates in a matrix
  };

  /// A prior factor on Key with Pose3 data type.
  typedef PriorFactor<Pose3> Prior;
  /// A factor to put constraints between two factors.
  typedef BetweenFactor<Pose3> Constraint;
  /// A hard constraint would enforce that the given key would have the input value in the results.
  typedef NonlinearEquality<Pose3> HardConstraint;

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @ingroup SLAM
   */
  struct Graph: public NonlinearFactorGraph {

    /// Adds a factor between keys of the same type
    void addPrior(Key i, const Pose3& p, const SharedNoiseModel& model);

    /// Creates a between factor between keys i and j with a noise model with Pos3 z in the graph
    void addConstraint(Key i1, Key i2, const Pose3& z, const SharedNoiseModel& model);

    /// Creates a hard constraint for key i with the given Pose3 p.
    void addHardConstraint(Key i, const Pose3& p);

    /// Optimize
    Values optimize(const Values& initialEstimate) {
      return LevenbergMarquardtOptimizer(*this, initialEstimate).optimize();
    }

    /// Return a Marginals object
    Marginals marginals(const Values& solution) const {
    	return Marginals(*this,solution);
    }
  };

} // pose3SLAM

/**
 * Backwards compatibility
 */
namespace gtsam {
	typedef pose3SLAM::Prior Pose3Prior;			///< Typedef for Prior class for backwards compatibility
	typedef pose3SLAM::Constraint Pose3Factor;		///< Typedef for Constraint class for backwards compatibility
	typedef pose3SLAM::Graph Pose3Graph;			///< Typedef for Graph class for backwards compatibility
}
