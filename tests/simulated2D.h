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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Point2.h>
#include "gtsam/base/OptionalJacobian.h"

// \namespace

namespace simulated2D {

  using namespace gtsam;

  // Simulated2D robots have no orientation, just a position

  /**
   *  Custom Values class that holds poses and points, mainly used as a convenience for MATLAB wrapper
   */
  class Values: public gtsam::Values {
  private:
    int nrPoses_, nrPoints_;

  public:
    typedef gtsam::Values Base;  ///< base class
    typedef boost::shared_ptr<Point2> sharedPoint;  ///< shortcut to shared Point type

    /// Constructor
    Values() : nrPoses_(0), nrPoints_(0) {
    }

    /// Copy constructor
    Values(const Base& base) :
        Base(base), nrPoses_(0), nrPoints_(0) {
    }

    /// Insert a pose
    void insertPose(Key i, const Point2& p) {
      insert(i, p);
      nrPoses_++;
    }

    /// Insert a point
    void insertPoint(Key j, const Point2& p) {
      insert(j, p);
      nrPoints_++;
    }

    /// Number of poses
    int nrPoses() const {
      return nrPoses_;
    }

    /// Number of points
    int nrPoints() const {
      return nrPoints_;
    }

    /// Return pose i
    Point2 pose(Key i) const {
      return at<Point2>(i);
    }

    /// Return point j
    Point2 point(Key j) const {
      return at<Point2>(j);
    }
  };

  /// Prior on a single pose
  inline Point2 prior(const Point2& x) {
    return x;
  }

  /// Prior on a single pose, optionally returns derivative
  inline Point2 prior(const Point2& x, OptionalJacobian<2,2> H = OptionalNone) {
    if (H) *H = I_2x2;
    return x;
  }

  /// odometry between two poses
  inline Point2 odo(const Point2& x1, const Point2& x2) {
    return x2 - x1;
  }

  /// odometry between two poses, optionally returns derivative
  inline Point2 odo(const Point2& x1, const Point2& x2, 
		  OptionalJacobian<2,2> H1 = OptionalNone, 
		  OptionalJacobian<2,2> H2 = OptionalNone) {
      if (H1) *H1 = -I_2x2;
      if (H2) *H2 = I_2x2;
      return x2 - x1;
  }

  /// measurement between landmark and pose
  inline Point2 mea(const Point2& x, const Point2& l) {
    return l - x;
  }

  /// measurement between landmark and pose, optionally returns derivative
  inline Point2 mea(const Point2& x, const Point2& l, OptionalJacobian<2,2> H1 =
    OptionalNone, OptionalMatrixType H2 = OptionalNone) {
      if (H1) *H1 = -I_2x2;
      if (H2) *H2 = I_2x2;
      return l - x;
  }

  /**
   *  Unary factor encoding a soft prior on a vector
   */
  template<class VALUE = Point2>
  class GenericPrior: public NoiseModelFactorN<VALUE> {
  public:
    typedef NoiseModelFactorN<VALUE> Base;  ///< base class
    typedef GenericPrior<VALUE> This;
    typedef boost::shared_ptr<GenericPrior<VALUE> > shared_ptr;
    typedef VALUE Pose; ///< shortcut to Pose type
	
    using Base::evaluateError;

    Pose measured_; ///< prior mean

    /// Create generic prior
    GenericPrior(const Pose& z, const SharedNoiseModel& model, Key key) :
      Base(model, key), measured_(z) {
    }

    /// Return error and optional derivative
    Vector evaluateError(const Pose& x, OptionalMatrixType H) const override {
      return (prior(x, H) - measured_);
    }

    ~GenericPrior() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  private:

    /// Default constructor
    GenericPrior() { }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  };

  /**
   * Binary factor simulating "odometry" between two Vectors
   */
  template<class VALUE = Point2>
  class GenericOdometry: public NoiseModelFactorN<VALUE, VALUE> {
  public:
    typedef NoiseModelFactorN<VALUE, VALUE> Base; ///< base class
    typedef GenericOdometry<VALUE> This;
    typedef boost::shared_ptr<GenericOdometry<VALUE> > shared_ptr;
    typedef VALUE Pose; ///< shortcut to Pose type

    using Base::evaluateError;

    Pose measured_; ///< odometry measurement

    /// Create odometry
    GenericOdometry(const Pose& measured, const SharedNoiseModel& model, Key i1, Key i2) :
          Base(model, i1, i2), measured_(measured) {
    }

    /// Evaluate error and optionally return derivatives
    Vector evaluateError(const Pose& x1, const Pose& x2,
        OptionalMatrixType H1, OptionalMatrixType H2) const override {
      return (odo(x1, x2, H1, H2) - measured_);
    }

    ~GenericOdometry() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  private:

    /// Default constructor
    GenericOdometry() { }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  };

  /**
   * Binary factor simulating "measurement" between two Vectors
   */
  template<class POSE, class LANDMARK>
  class GenericMeasurement: public NoiseModelFactorN<POSE, LANDMARK> {
  public:
    typedef NoiseModelFactorN<POSE, LANDMARK> Base;  ///< base class
    typedef GenericMeasurement<POSE, LANDMARK> This;
    typedef boost::shared_ptr<GenericMeasurement<POSE, LANDMARK> > shared_ptr;
    typedef POSE Pose; ///< shortcut to Pose type
    typedef LANDMARK Landmark; ///< shortcut to Landmark type

    using Base::evaluateError;

    Landmark measured_; ///< Measurement

    /// Create measurement factor
    GenericMeasurement(const Landmark& measured, const SharedNoiseModel& model, Key i, Key j) :
          Base(model, i, j), measured_(measured) {
    }

    /// Evaluate error and optionally return derivatives
    Vector evaluateError(const Pose& x1, const Landmark& x2,
        OptionalMatrixType H1, OptionalMatrixType H2) const override {
      return (mea(x1, x2, H1, H2) - measured_);
    }

    ~GenericMeasurement() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  private:

    /// Default constructor
    GenericMeasurement() { }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  };

  /** Typedefs for regular use */
  typedef GenericPrior<Point2> Prior;
  typedef GenericOdometry<Point2> Odometry;
  typedef GenericMeasurement<Point2, Point2> Measurement;

  // Specialization of a graph for this example domain
  // TODO: add functions to add factor types
  class Graph : public NonlinearFactorGraph {
  public:
    Graph() {}
  };

} // namespace simulated2D

/// traits
namespace gtsam {
template<class POSE, class LANDMARK>
struct traits<simulated2D::GenericMeasurement<POSE, LANDMARK> > : Testable<
    simulated2D::GenericMeasurement<POSE, LANDMARK> > {
};

template<>
struct traits<simulated2D::Values> : public Testable<simulated2D::Values> {
};
}

