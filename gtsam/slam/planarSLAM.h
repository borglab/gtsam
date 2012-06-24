/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  planarSLAM.h
 *  @brief bearing/range measurements in 2D plane
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>

// Use planarSLAM namespace for specific SLAM instance
namespace planarSLAM {

  using namespace gtsam;

  /*
   * Values class, inherited from pose2SLAM::Values, mainly used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Values: public pose2SLAM::Values {

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
    	pose2SLAM::Values(values) {
    }

    /// get a point
    Point2 point(Key j) const { return at<Point2>(j); }

    /// insert a point
    void insertPoint(Key j, const Point2& point) { insert(j, point); }

    /// update a point
    void updatePoint(Key j, const Point2& point) { update(j, point); }

    /// get all [x,y] coordinates in a 2*n matrix
    Matrix points() const;
};

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public pose2SLAM::Graph {

    /// Default constructor for a NonlinearFactorGraph
    Graph(){}

    /// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph):
  		pose2SLAM::Graph(graph) {}

    /// Creates a hard constraint on a point
    void addPointConstraint(Key j, const Point2& p);

    /// Adds a prior with mean p and given noise model on point j
    void addPointPrior(Key j, const Point2& p, const SharedNoiseModel& model);

    /// Creates a bearing measurement from pose i to point j
    void addBearing(Key i, Key j, const Rot2& bearing, const SharedNoiseModel& model);

    /// Creates a range measurement from pose i to point j
    void addRange(Key i, Key k, double range, const SharedNoiseModel& model);

    /// Creates a range/bearing measurement from pose i to point j
    void addBearingRange(Key i, Key j, const Rot2& bearing, double range, const SharedNoiseModel& model);
  };

} // planarSLAM


/**
 * Backwards compatibility and wrap use only, avoid using
 */
namespace planarSLAM {
	typedef gtsam::NonlinearEquality<Pose2> Constraint; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::PriorFactor<Pose2> Prior; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::BetweenFactor<Pose2> Odometry; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::BearingFactor<Pose2, Point2> Bearing; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::RangeFactor<Pose2, Point2> Range; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::BearingRangeFactor<Pose2, Point2> BearingRange; ///< \deprecated typedef for backwards compatibility
}


