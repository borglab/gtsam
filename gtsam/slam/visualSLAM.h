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
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace gtsam {

	namespace visualSLAM {

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef TypedSymbol<Pose3,'x'> PoseKey;									///< The key type used for poses
  typedef TypedSymbol<Point3,'l'> PointKey;								///< The key type used for points
  typedef Values<PoseKey> PoseValues;									///< Values used for poses
  typedef Values<PointKey> PointValues;								///< Values used for points
  typedef TupleValues2<PoseValues, PointValues> Values;		///< Values data structure
  typedef boost::shared_ptr<Values> shared_values;				///< shared pointer to values data structure

  typedef NonlinearEquality<Values, PoseKey> PoseConstraint;	 ///< put a hard constraint on a pose
  typedef NonlinearEquality<Values, PointKey> PointConstraint; ///< put a hard constraint on a point
  typedef PriorFactor<Values, PoseKey> PosePrior;							 ///< put a soft prior on a Pose
  typedef PriorFactor<Values, PointKey> PointPrior;						 ///< put a soft prior on a point
  typedef RangeFactor<Values, PoseKey, PointKey> RangeFactor;  ///< put a factor on the range from a pose to a point
  
  /// monocular and stereo camera typedefs for general use
  typedef GenericProjectionFactor<Values, PointKey, PoseKey> ProjectionFactor;
  typedef GenericStereoFactor<Values, PoseKey, PointKey> StereoFactor;

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  class Graph: public NonlinearFactorGraph<Values> {

  public:
  	/// shared pointer to this type of graph
    typedef boost::shared_ptr<Graph> shared_graph;

    /// default constructor is empty graph
    Graph() {
    }

    /// print out graph
    void print(const std::string& s = "") const {
      NonlinearFactorGraph<Values>::print(s);
    }

    /// check if two graphs are equal
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph<Values>::equals(p, tol);
    }

    /**
     *  Add a projection factor measurement (monocular)
     *  @param z the measurement
     *  @param model the noise model for the measurement
     *  @param i index of camera
     *  @param j index of point
     *  @param K shared pointer to calibration object
     */
    void addMeasurement(const Point2& z, const SharedNoiseModel& model,
        PoseKey i, PointKey j, const shared_ptrK& K) {
      boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(z, model, i, j, K));
      push_back(factor);
    }

    /**
     *  Add a constraint on a pose (for now, *must* be satisfied in any Values)
     *  @param j index of camera
     *  @param p to which pose to constrain it to
     */
    void addPoseConstraint(int j, const Pose3& p = Pose3()) {
      boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(j, p));
      push_back(factor);
    }

    /**
     *  Add a constraint on a point (for now, *must* be satisfied in any Values)
     *  @param j index of landmark
     *  @param p point around which soft prior is defined
     */
    void addPointConstraint(int j, const Point3& p = Point3()) {
      boost::shared_ptr<PointConstraint> factor(new PointConstraint(j, p));
      push_back(factor);
    }

    /**
     *  Add a prior on a pose
     *  @param j index of camera
     *  @param p around which soft prior is defined
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(int j, const Pose3& p = Pose3(), const SharedNoiseModel& model = noiseModel::Unit::Create(6)) {
      boost::shared_ptr<PosePrior> factor(new PosePrior(j, p, model));
      push_back(factor);
    }

    /**
     *  Add a prior on a landmark
     *  @param j index of landmark
     *  @param p to which point to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(int j, const Point3& p = Point3(), const SharedNoiseModel& model = noiseModel::Unit::Create(3)) {
      boost::shared_ptr<PointPrior> factor(new PointPrior(j, p, model));
      push_back(factor);
    }

    /**
     *  Add a range prior to a landmark
     *  @param i index of pose
     *  @param j index of landmark
     *  @param range approximate range to landmark
     *  @param model uncertainty model of this prior
     */
    void addRangeFactor(PoseKey i, PointKey j, double range, const SharedNoiseModel& model = noiseModel::Unit::Create(1)) {
      push_back(boost::shared_ptr<RangeFactor>(new RangeFactor(i, j, range, model)));
    }

  }; // Graph

  /// typedef for Optimizer. The current default will use the multi-frontal solver
  typedef NonlinearOptimizer<Graph, Values> Optimizer;

} } // namespaces
