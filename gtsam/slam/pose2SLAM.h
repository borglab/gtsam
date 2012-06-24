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
#include <gtsam/nonlinear/EasyFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/geometry/Pose2.h>

// Use pose2SLAM namespace for specific SLAM instance
namespace pose2SLAM {

	using namespace gtsam;

  /*
   * Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
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
     * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
     * @param n number of poses
     * @param R radius of circle
     * @param c character to use for keys
     * @return circle of n 2D poses
     */
    static Values Circle(size_t n, double R);

    /// insert a pose
    void insertPose(Key i, const Pose2& pose) { insert(i, pose); }

    /// update a pose
    void updatePose(Key i, const Pose2& pose) { update(i, pose); }

    /// get a pose
    Pose2 pose(Key i) const { return at<Pose2>(i); }

    /// get all [x,y,theta] coordinates in a 3*m matrix
    Matrix poses() const;
  };

  /**
   * List of typedefs for factors
   */

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public EasyFactorGraph {

	  typedef boost::shared_ptr<Graph> shared_ptr;

	  /// Default constructor
    Graph(){}

    /// Copy constructor given any other NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph):
    	EasyFactorGraph(graph) {}

    /// Creates a hard constraint for key i with the given Pose2 p.
    void addPoseConstraint(Key i, const Pose2& p);

    /// Adds a Pose2 prior with mean p and given noise model on pose i
    void addPosePrior(Key i, const Pose2& p, const SharedNoiseModel& model);

    /// Creates an relative pose factor between poses with keys i1 and i2
    void addRelativePose(Key i1, Key i2, const Pose2& z, const SharedNoiseModel& model);
  };

} // pose2SLAM

/**
 * Backwards compatibility and wrap use only, avoid using
 */
namespace pose2SLAM {
	typedef gtsam::NonlinearEquality<Pose2> HardConstraint;	 ///< \deprecated typedef for backwards compatibility
	typedef gtsam::PriorFactor<Pose2> Prior;	                 ///< \deprecated typedef for backwards compatibility
	typedef gtsam::BetweenFactor<Pose2> Odometry;	           ///< \deprecated typedef for backwards compatibility
}
