/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file visualSLAM.h
 *  @brief Basic typedefs for general VisualSLAM problems. Useful for monocular and stereo systems.
 *  @date Jan 14, 2010
 *  @author Richard Roberts
 *  @author Chris Beall
 */

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/geometry/SimpleCamera.h>


namespace visualSLAM {

	using namespace gtsam;

  /// Convenience function for constructing a pose key
  inline Symbol PoseKey(Index j) { return Symbol('x', j); }

  /// Convenience function for constructing a pose key
  inline Symbol PointKey(Index j) { return Symbol('l', j); }

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef NonlinearEquality<Pose3> PoseConstraint;	 ///< put a hard constraint on a pose
  typedef NonlinearEquality<Point3> PointConstraint; ///< put a hard constraint on a point
  typedef PriorFactor<Pose3> PosePrior;							 ///< put a soft prior on a Pose
  typedef PriorFactor<Point3> PointPrior;						 ///< put a soft prior on a point
  typedef RangeFactor<Pose3, Point3> RangeFactor;  ///< put a factor on the range from a pose to a point
  
  /// monocular and stereo camera typedefs for general use
  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  typedef GenericStereoFactor<Pose3, Point3> StereoFactor;

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  class Graph: public NonlinearFactorGraph {

  public:
  	/// shared pointer to this type of graph
    typedef boost::shared_ptr<Graph> shared_graph;

    /// default constructor is empty graph
    Graph() {
    }

    /// print out graph
    void print(const std::string& s = "") const {
      NonlinearFactorGraph::print(s);
    }

    /// check if two graphs are equal
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph::equals(p, tol);
    }

    /**
     *  Add a projection factor measurement (monocular)
     *  @param measured the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the camera pose
     *  @param pointKey variable key for the landmark
     *  @param K shared pointer to calibration object
     */
    void addMeasurement(const Point2& measured, const SharedNoiseModel& model,
        Index poseKey, Index pointKey, const shared_ptrK& K) {
      boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(measured, model, PoseKey(poseKey), PointKey(pointKey), K));
      push_back(factor);
    }

    /**
     *  Add a constraint on a pose (for now, *must* be satisfied in any Values)
     *  @param key variable key of the camera pose
     *  @param p to which pose to constrain it to
     */
    void addPoseConstraint(Index poseKey, const Pose3& p = Pose3()) {
      boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(PoseKey(poseKey), p));
      push_back(factor);
    }

    /**
     *  Add a constraint on a point (for now, *must* be satisfied in any Values)
     *  @param key variable key of the landmark
     *  @param p point around which soft prior is defined
     */
    void addPointConstraint(Index pointKey, const Point3& p = Point3()) {
      boost::shared_ptr<PointConstraint> factor(new PointConstraint(PointKey(pointKey), p));
      push_back(factor);
    }

    /**
     *  Add a prior on a pose
     *  @param key variable key of the camera pose
     *  @param p around which soft prior is defined
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(Index poseKey, const Pose3& p = Pose3(), const SharedNoiseModel& model = noiseModel::Unit::Create(6)) {
      boost::shared_ptr<PosePrior> factor(new PosePrior(PoseKey(poseKey), p, model));
      push_back(factor);
    }

    /**
     *  Add a prior on a landmark
     *  @param key variable key of the landmark
     *  @param p to which point to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(Index pointKey, const Point3& p = Point3(), const SharedNoiseModel& model = noiseModel::Unit::Create(3)) {
      boost::shared_ptr<PointPrior> factor(new PointPrior(PointKey(pointKey), p, model));
      push_back(factor);
    }

    /**
     *  Add a range prior to a landmark
     *  @param poseKey variable key of the camera pose
     *  @param pointKey variable key of the landmark
     *  @param range approximate range to landmark
     *  @param model uncertainty model of this prior
     */
    void addRangeFactor(Index poseKey, Index pointKey, double range, const SharedNoiseModel& model = noiseModel::Unit::Create(1)) {
      push_back(boost::shared_ptr<RangeFactor>(new RangeFactor(PoseKey(poseKey), PointKey(pointKey), range, model)));
    }

  }; // Graph

  /// typedef for Optimizer. The current default will use the multi-frontal solver
  typedef NonlinearOptimizer<Graph> Optimizer;

} // namespaces
