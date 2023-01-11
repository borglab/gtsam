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
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "gtsam/base/OptionalJacobian.h"

// \namespace
namespace simulated2DOriented {

  using namespace gtsam;

  /// Specialized Values structure with syntactic sugar for
  /// compatibility with matlab
  class Values: public gtsam::Values {
    int nrPoses_, nrPoints_;
  public:
    Values() : nrPoses_(0), nrPoints_(0)  {}

    /// insert a pose
    void insertPose(Key i, const Pose2& p) {
      insert(i, p);
      nrPoses_++;
    }

    /// insert a landmark
    void insertPoint(Key j, const Point2& p) {
      insert(j, p);
      nrPoints_++;
    }

    int nrPoses() const {  return nrPoses_;  } ///< nr of poses
    int nrPoints() const { return nrPoints_;  } ///< nr of landmarks

    Pose2 pose(Key i) const { return at<Pose2>(i);  } ///< get a pose
    Point2 point(Key j) const { return at<Point2>(j); } ///< get a landmark
  };

  //TODO:: point prior is not implemented right now

  /// Prior on a single pose
  inline Pose2 prior(const Pose2& x) {
    return x;
  }

  /// Prior on a single pose, optional derivative version
  Pose2 prior(const Pose2& x, OptionalJacobian<3,3> H = OptionalNone) {
    if (H) *H = I_3x3;
    return x;
  }

  /// odometry between two poses
  inline Pose2 odo(const Pose2& x1, const Pose2& x2) {
    return x1.between(x2);
  }

  /// odometry between two poses, optional derivative version
  Pose2 odo(const Pose2& x1, const Pose2& x2, OptionalJacobian<3,3> H1 =
      OptionalNone, OptionalJacobian<3,3> H2 = OptionalNone) {
    return x1.between(x2, H1, H2);
  }

  /// Unary factor encoding a soft prior on a vector
  template<class VALUE = Pose2>
  struct GenericPosePrior: public NoiseModelFactorN<VALUE> {

    Pose2 measured_; ///< measurement

    /// Create generic pose prior
    GenericPosePrior(const Pose2& measured, const SharedNoiseModel& model, Key key) :
      NoiseModelFactorN<VALUE>(model, key), measured_(measured) {
    }

    /// Evaluate error and optionally derivative
    Vector evaluateError(const Pose2& x, OptionalMatrixType H =
        OptionalNone) const {
      return measured_.localCoordinates(prior(x, H));
    }

  };

  /**
   * Binary factor simulating "odometry" between two Vectors
   */
  template<class VALUE = Pose2>
  struct GenericOdometry: public NoiseModelFactorN<VALUE, VALUE> {
    Pose2 measured_;   ///< Between measurement for odometry factor

    typedef GenericOdometry<VALUE> This;

    // Provide access to the Matrix& version of evaluateError:
	  using NoiseModelFactor2<VALUE, VALUE>::evaluateError;

    /**
     * Creates an odometry factor between two poses
     */
    GenericOdometry(const Pose2& measured, const SharedNoiseModel& model,
        Key i1, Key i2) :
          NoiseModelFactorN<VALUE, VALUE>(model, i1, i2), measured_(measured) {
    }

    ~GenericOdometry() override {}

    /// Evaluate error and optionally derivative
    Vector evaluateError(const VALUE& x1, const VALUE& x2,
        OptionalMatrixType H1, OptionalMatrixType H2) const override {
      return measured_.localCoordinates(odo(x1, x2, H1, H2));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  };

  typedef GenericOdometry<Pose2> Odometry;

  /// Graph specialization for syntactic sugar use with matlab
  class Graph : public NonlinearFactorGraph {
  public:
    Graph() {}
    // TODO: add functions to add factors
  };

} // namespace simulated2DOriented
