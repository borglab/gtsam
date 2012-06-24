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
#include <gtsam/nonlinear/EasyFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/geometry/Pose3.h>

/// Use pose3SLAM namespace for specific SLAM instance
namespace pose3SLAM {

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
     * Create a circle of n 3D poses tangent to circle of radius R, first pose at (R,0)
     * @param n number of poses
     * @param R radius of circle
     * @return circle of n 3D poses
     */
    static Values Circle(size_t n, double R);

    /// insert a pose
    void insertPose(Key i, const Pose3& pose) { insert(i, pose); }

    /// update a pose
    void updatePose(Key i, const Pose3& pose) { update(i, pose); }

    /// get a pose
    Pose3 pose(Key i) const { return at<Pose3>(i); }

    Matrix translations() const; ///< get all pose translations coordinates in a matrix
  };

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public EasyFactorGraph {

	  /// Default constructor
    Graph(){}

    /// Copy constructor given any other NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph):
    	EasyFactorGraph(graph) {}

    /**
     *  Add a prior on a pose
     *  @param key variable key of the camera pose
     *  @param p around which soft prior is defined
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(Key poseKey, const Pose3& p = Pose3(), const SharedNoiseModel& model = noiseModel::Unit::Create(6));

    /**
     *  Add a constraint on a pose (for now, *must* be satisfied in any Values)
     *  @param key variable key of the camera pose
     *  @param p to which pose to constrain it to
     */
    void addPoseConstraint(Key poseKey, const Pose3& p = Pose3());

    /**
     *  Add a relative pose measurement between two poses
     *  @param x1 variable key of the first camera pose
     *  @param x2 variable key of the second camera pose
     *  @param relative pose measurement from x1 to x2 (x1.between(x2))
     *  @param model uncertainty model of this measurement
     */
    void addRelativePose(Key x1, Key x2, const Pose3& z, const SharedNoiseModel& model);
  };

} // pose3SLAM

/**
 * Backwards compatibility and wrap use only, avoid using
 */
namespace pose3SLAM {
  typedef gtsam::PriorFactor<Pose3> Prior;                ///< \deprecated typedef for backwards compatibility
  typedef gtsam::BetweenFactor<Pose3> Constraint;         ///< \deprecated typedef for backwards compatibility
  typedef gtsam::NonlinearEquality<Pose3> HardConstraint; ///< \deprecated typedef for backwards compatibility
}
namespace gtsam {
	typedef pose3SLAM::Prior Pose3Prior;       ///< \deprecated typedef for backwards compatibility
	typedef pose3SLAM::Constraint Pose3Factor; ///< \deprecated typedef for backwards compatibility
	typedef pose3SLAM::Graph Pose3Graph;       ///< \deprecated typedef for backwards compatibility
}

