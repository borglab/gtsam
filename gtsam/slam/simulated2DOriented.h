/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// \namespace
namespace simulated2DOriented {

  using namespace gtsam;

  /// Convenience function for constructing a pose key
  inline Symbol PoseKey(Index j) { return Symbol('x', j); }

  /// Convenience function for constructing a landmark key
  inline Symbol PointKey(Index j) { return Symbol('l', j); }

  /// Specialized Values structure with syntactic sugar for
  /// compatibility with matlab
  class Values: public gtsam::Values {
  	int nrPoses_, nrPoints_;
  public:
    Values() : nrPoses_(0), nrPoints_(0)  {}

    void insertPose(Index j, const Pose2& p) {
      insert(PoseKey(j), p);
      nrPoses_++;
    }

    void insertPoint(Index j, const Point2& p) {
      insert(PointKey(j), p);
      nrPoints_++;
    }

    int nrPoses() const {	return nrPoses_;	}
    int nrPoints() const { return nrPoints_;	}

    Pose2 pose(Index j) const { return at<Pose2>(PoseKey(j));	}
    Point2 point(Index j) const { return at<Point2>(PointKey(j)); }
  };

  //TODO:: point prior is not implemented right now

  /// Prior on a single pose
  inline Pose2 prior(const Pose2& x) {
    return x;
  }

  /// Prior on a single pose, optional derivative version
  Pose2 prior(const Pose2& x, boost::optional<Matrix&> H = boost::none);

  /// odometry between two poses
  inline Pose2 odo(const Pose2& x1, const Pose2& x2) {
    return x1.between(x2);
  }

  /// odometry between two poses, optional derivative version
  Pose2 odo(const Pose2& x1, const Pose2& x2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none);

  /// Unary factor encoding a soft prior on a vector
  template<class VALUE = Pose2>
  struct GenericPosePrior: public NonlinearFactor1<VALUE> {

    Pose2 measured_; ///< measurement

    /// Create generic pose prior
    GenericPosePrior(const Pose2& measured, const SharedNoiseModel& model, const Symbol& key) :
      NonlinearFactor1<VALUE>(model, key), measured_(measured) {
    }

    /// Evaluate error and optionally derivative
    Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H =
        boost::none) const {
      return measured_.localCoordinates(prior(x, H));
    }

  };

  /**
   * Binary factor simulating "odometry" between two Vectors
   */
  template<class VALUE = Pose2>
  struct GenericOdometry: public NonlinearFactor2<VALUE, VALUE> {
    Pose2 measured_;   ///< Between measurement for odometry factor

    /**
     * Creates an odometry factor between two poses
     */
    GenericOdometry(const Pose2& measured, const SharedNoiseModel& model,
        const Symbol& i1, const Symbol& i2) :
          NonlinearFactor2<VALUE, VALUE>(model, i1, i2), measured_(measured) {
    }

    /// Evaluate error and optionally derivative
    Vector evaluateError(const VALUE& x1, const VALUE& x2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const {
      return measured_.localCoordinates(odo(x1, x2, H1, H2));
    }

  };

  typedef GenericOdometry<Pose2> Odometry;

  /// Graph specialization for syntactic sugar use with matlab
  class Graph : public NonlinearFactorGraph {
  public:
    Graph() {}
    // TODO: add functions to add factors
  };

} // namespace simulated2DOriented
