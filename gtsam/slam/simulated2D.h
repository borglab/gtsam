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

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// \namespace

namespace simulated2D {

  using namespace gtsam;

  // Simulated2D robots have no orientation, just a position
  typedef TypedSymbol<Point2, 'x'> PoseKey;
  typedef TypedSymbol<Point2, 'l'> PointKey;

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
    void insertPose(const simulated2D::PoseKey& i, const Point2& p) {
      insert(i, p);
      nrPoses_++;
    }

    /// Insert a point
    void insertPoint(const simulated2D::PointKey& j, const Point2& p) {
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
    Point2 pose(const simulated2D::PoseKey& i) const {
      return (*this)[i];
    }

    /// Return point j
    Point2 point(const simulated2D::PointKey& j) const {
      return (*this)[j];
    }
  };


  /// Prior on a single pose
  inline Point2 prior(const Point2& x) {
    return x;
  }

  /// Prior on a single pose, optionally returns derivative
  Point2 prior(const Point2& x, boost::optional<Matrix&> H = boost::none);

  /// odometry between two poses
  inline Point2 odo(const Point2& x1, const Point2& x2) {
    return x2 - x1;
  }

  /// odometry between two poses, optionally returns derivative
  Point2 odo(const Point2& x1, const Point2& x2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none);

  /// measurement between landmark and pose
  inline Point2 mea(const Point2& x, const Point2& l) {
    return l - x;
  }

  /// measurement between landmark and pose, optionally returns derivative
  Point2 mea(const Point2& x, const Point2& l, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none);

  /**
   *  Unary factor encoding a soft prior on a vector
   */
  template<class KEY = PoseKey>
  class GenericPrior: public NonlinearFactor1<KEY> {
  public:
    typedef NonlinearFactor1<KEY> Base;  ///< base class
    typedef boost::shared_ptr<GenericPrior<KEY> > shared_ptr;
    typedef typename KEY::Value Pose; ///< shortcut to Pose type

    Pose z_; ///< prior mean

    /// Create generic prior
    GenericPrior(const Pose& z, const SharedNoiseModel& model, const KEY& key) :
      NonlinearFactor1<KEY>(model, key), z_(z) {
    }

    /// Return error and optional derivative
    Vector evaluateError(const Pose& x, boost::optional<Matrix&> H =
        boost::none) const {
      return (prior(x, H) - z_).vector();
    }

  private:

    /// Default constructor
    GenericPrior() {
    }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }
  };

  /**
   * Binary factor simulating "odometry" between two Vectors
   */
  template<class KEY = PoseKey>
  class GenericOdometry: public NonlinearFactor2<KEY, KEY> {
  public:
    typedef NonlinearFactor2<KEY, KEY> Base; ///< base class
    typedef boost::shared_ptr<GenericOdometry<KEY> > shared_ptr;
    typedef typename KEY::Value Pose; ///< shortcut to Pose type

    Pose z_; ///< odometry measurement

    /// Create odometry
    GenericOdometry(const Pose& z, const SharedNoiseModel& model,
        const KEY& i1, const KEY& i2) :
          NonlinearFactor2<KEY, KEY>(model, i1, i2), z_(z) {
    }

    /// Evaluate error and optionally return derivatives
    Vector evaluateError(const Pose& x1, const Pose& x2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const {
      return (odo(x1, x2, H1, H2) - z_).vector();
    }

  private:

    /// Default constructor
    GenericOdometry() {
    }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }
  };

  /**
   * Binary factor simulating "measurement" between two Vectors
   */
  template<class XKEY = PoseKey, class LKEY = PointKey>
  class GenericMeasurement: public NonlinearFactor2<XKEY, LKEY> {
  public:
    typedef NonlinearFactor2<XKEY, LKEY> Base;  ///< base class
    typedef boost::shared_ptr<GenericMeasurement<XKEY, LKEY> > shared_ptr;
    typedef typename XKEY::Value Pose; ///< shortcut to Pose type
    typedef typename LKEY::Value Point; ///< shortcut to Point type

    Point z_; ///< Measurement

    /// Create measurement factor
    GenericMeasurement(const Point& z, const SharedNoiseModel& model,
        const XKEY& i, const LKEY& j) :
          NonlinearFactor2<XKEY, LKEY>(model, i, j), z_(z) {
    }

    /// Evaluate error and optionally return derivatives
    Vector evaluateError(const Pose& x1, const Point& x2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const {
      return (mea(x1, x2, H1, H2) - z_).vector();
    }

  private:

    /// Default constructor
    GenericMeasurement() {
    }

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }
  };

  /** Typedefs for regular use */
  typedef GenericPrior<PoseKey> Prior;
  typedef GenericOdometry<PoseKey> Odometry;
  typedef GenericMeasurement<PoseKey, PointKey> Measurement;

  // Specialization of a graph for this example domain
  // TODO: add functions to add factor types
  class Graph : public NonlinearFactorGraph {
  public:
    Graph() {}
  };

} // namespace simulated2D
