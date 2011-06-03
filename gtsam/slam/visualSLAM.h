/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * visualSLAM.h
 *
 *  Created on: Jan 14, 2010
 *      Author: Richard Roberts and Chris Beall
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

namespace gtsam {

	namespace visualSLAM {

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef TypedSymbol<Pose3,'x'> PoseKey;
  typedef TypedSymbol<Point3,'l'> PointKey;
  typedef LieValues<PoseKey> PoseValues;
  typedef LieValues<PointKey> PointValues;
  typedef TupleValues2<PoseValues, PointValues> Values;
  typedef boost::shared_ptr<Values> shared_values;

  typedef NonlinearEquality<Values, PoseKey> PoseConstraint;
  typedef NonlinearEquality<Values, PointKey> PointConstraint;
  typedef PriorFactor<Values, PoseKey> PosePrior;
  typedef PriorFactor<Values, PointKey> PointPrior;
  
  // Typedef for general use
  typedef GenericProjectionFactor<Values, PointKey, PoseKey> ProjectionFactor;
  typedef GenericStereoFactor<Values, PoseKey, PointKey> StereoFactor;

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  class Graph: public NonlinearFactorGraph<Values> {

  public:

    typedef boost::shared_ptr<Graph> shared_graph;

    /** default constructor is empty graph */
    Graph() {
    }

    /** print out graph */
    void print(const std::string& s = "") const {
      NonlinearFactorGraph<Values>::print(s);
    }

    /** equals */
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph<Values>::equals(p, tol);
    }

    /**
     *  Add a measurement
     *  @param j index of camera
     *  @param p to which pose to constrain it to
     */
    void addMeasurement(const Point2& z, const SharedGaussian& model,
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
     *  @param p to which point to constrain it to
     */
    void addPointConstraint(int j, const Point3& p = Point3()) {
      boost::shared_ptr<PointConstraint> factor(new PointConstraint(j, p));
      push_back(factor);
    }

    /**
     *  Add a prior on a pose
     *  @param j index of camera
     *  @param p to which pose to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(int j, const Pose3& p = Pose3(), const SharedGaussian& model = noiseModel::Unit::Create(1)) {
      boost::shared_ptr<PosePrior> factor(new PosePrior(j, p, model));
      push_back(factor);
    }

    /**
     *  Add a prior on a landmark
     *  @param j index of landmark
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(int j, const Point3& p = Point3(), const SharedGaussian& model = noiseModel::Unit::Create(1)) {
      boost::shared_ptr<PointPrior> factor(new PointPrior(j, p, model));
      push_back(factor);
    }

  }; // Graph

  // Optimizer
  typedef NonlinearOptimizer<Graph, Values> Optimizer;

} } // namespaces
