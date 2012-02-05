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

  // The types that take an oriented pose2 rather than point2
  typedef TypedSymbol<Point2, 'l'> PointKey;
  typedef TypedSymbol<Pose2, 'x'> PoseKey;

  /// Specialized Values structure with syntactic sugar for
  /// compatibility with matlab
  class Values: public gtsam::Values {
  	int nrPoses_, nrPoints_;
  public:
    Values() : nrPoses_(0), nrPoints_(0)  {}

    void insertPose(const PoseKey& i, const Pose2& p) {
      insert(i, p);
      nrPoses_++;
    }

    void insertPoint(const PointKey& j, const Point2& p) {
      insert(j, p);
      nrPoints_++;
    }

    int nrPoses() const {	return nrPoses_;	}
    int nrPoints() const { return nrPoints_;	}

    Pose2 pose(const PoseKey& i) const { return (*this)[i];	}
    Point2 point(const PointKey& j) const { return (*this)[j]; }
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
		template<class Key = PoseKey>
		struct GenericPosePrior: public NonlinearFactor1<Key> {

    Pose2 z_; ///< measurement

    /// Create generic pose prior
    GenericPosePrior(const Pose2& z, const SharedNoiseModel& model,
        const Key& key) :
					NonlinearFactor1<Key>(model, key), z_(z) {
    }

    /// Evaluate error and optionally derivative
    Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H =
        boost::none) const {
      return z_.localCoordinates(prior(x, H));
    }

  };

  /**
   * Binary factor simulating "odometry" between two Vectors
   */
		template<class KEY = PoseKey>
		struct GenericOdometry: public NonlinearFactor2<KEY, KEY> {
    Pose2 z_;   ///< Between measurement for odometry factor

    /**
     * Creates an odometry factor between two poses
     */
    GenericOdometry(const Pose2& z, const SharedNoiseModel& model,
        const KEY& i1, const KEY& i2) :
					NonlinearFactor2<KEY, KEY>(model, i1, i2), z_(z) {
    }

    /// Evaluate error and optionally derivative
    Vector evaluateError(const Pose2& x1, const Pose2& x2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const {
      return z_.localCoordinates(odo(x1, x2, H1, H2));
    }

  };

		typedef GenericOdometry<PoseKey> Odometry;

  /// Graph specialization for syntactic sugar use with matlab
		class Graph : public NonlinearFactorGraph {
  public:
    Graph() {}
    // TODO: add functions to add factors
  };

} // namespace simulated2DOriented
