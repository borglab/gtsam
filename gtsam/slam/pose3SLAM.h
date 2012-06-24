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
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
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

    /// check if value with specified key exists
    bool exists(Key i) const { return gtsam::Values::exists(i); }

    Matrix translations() const; ///< get all pose translations coordinates in a matrix
  };

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public NonlinearFactorGraph {

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

    /**
     *  Optimize the graph
     *  @param initialEstimate initial estimate of all variables in the graph
     *  @param pointKey variable key of the landmark
     *  @param range approximate range to landmark
     *  @param model uncertainty model of this prior
     */
    Values optimize(const Values& initialEstimate, size_t verbosity=NonlinearOptimizerParams::SILENT) const;

    /**
     *  Setup and return a LevenbargMarquardtOptimizer
     *  @param initialEstimate initial estimate of all variables in the graph
     *  @param parameters optimizer's parameters
     *  @return a LevenbergMarquardtOptimizer object, which you can use to control the optimization process
     */
    LevenbergMarquardtOptimizer optimizer(const Values& initialEstimate,
				const LevenbergMarquardtParams& parameters = LevenbergMarquardtParams()) const {
			return LevenbergMarquardtOptimizer((*this), initialEstimate, parameters);
		}

    /// Return a Marginals object
    Marginals marginals(const Values& solution) const {
      return Marginals(*this,solution);
    }

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

