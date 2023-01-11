/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactorRollingShutter.h
 * @brief Basic projection factor for rolling shutter cameras
 * @author Yotam Stern
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dllexport.h>

#include <boost/optional.hpp>

namespace gtsam {

/**
 * Non-linear factor for 2D projection measurement obtained using a rolling
 * shutter camera. The calibration is known here. This version takes rolling
 * shutter information into account as follows: consider two consecutive poses A
 * and B, and a Point2 measurement taken starting at time A using a rolling
 * shutter camera. Pose A has timestamp t_A, and Pose B has timestamp t_B. The
 * Point2 measurement has timestamp t_p (with t_A <= t_p <= t_B) corresponding
 * to the time of exposure of the row of the image the pixel belongs to. Let us
 * define the alpha = (t_p - t_A) / (t_B - t_A), we will use the pose
 * interpolated between A and B by the alpha to project the corresponding
 * landmark to Point2.
 * @ingroup slam
 */

class GTSAM_UNSTABLE_EXPORT ProjectionFactorRollingShutter
    : public NoiseModelFactorN<Pose3, Pose3, Point3> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;  ///< 2D measurement
  double alpha_;     ///< interpolation parameter in [0,1] corresponding to the
                     ///< point2 measurement
  boost::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object
  boost::optional<Pose3>
      body_P_sensor_;  ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;  ///< If true, rethrows Cheirality exceptions (default:
                          ///< false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions
                            ///< (default: false)

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor3<Pose3, Pose3, Point3> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;


  /// shorthand for this class
  typedef ProjectionFactorRollingShutter This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  ProjectionFactorRollingShutter()
      : measured_(0, 0),
        alpha_(0),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor
   * @param measured is the 2-dimensional pixel location of point in the image
   * (the measurement)
   * @param alpha in [0,1] is the rolling shutter parameter for the measurement
   * @param model is the noise model
   * @param poseKey_a is the key of the first camera
   * @param poseKey_b is the key of the second camera
   * @param pointKey is the key of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity)
   */
  ProjectionFactorRollingShutter(
      const Point2& measured, double alpha, const SharedNoiseModel& model,
      Key poseKey_a, Key poseKey_b, Key pointKey,
      const boost::shared_ptr<Cal3_S2>& K,
      boost::optional<Pose3> body_P_sensor = boost::none)
      : Base(model, poseKey_a, poseKey_b, pointKey),
        measured_(measured),
        alpha_(alpha),
        K_(K),
        body_P_sensor_(body_P_sensor),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor with exception-handling flags
   * @param measured is the 2-dimensional pixel location of point in the image
   * (the measurement)
   * @param alpha in [0,1] is the rolling shutter parameter for the measurement
   * @param model is the noise model
   * @param poseKey_a is the key of the first camera
   * @param poseKey_b is the key of the second camera
   * @param pointKey is the key of the landmark
   * @param K shared pointer to the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are
   * rethrown
   * @param verboseCheirality determines whether exceptions are printed for
   * Cheirality
   * @param body_P_sensor is the transform from body to sensor frame  (default
   * identity)
   */
  ProjectionFactorRollingShutter(
      const Point2& measured, double alpha, const SharedNoiseModel& model,
      Key poseKey_a, Key poseKey_b, Key pointKey,
      const boost::shared_ptr<Cal3_S2>& K, bool throwCheirality,
      bool verboseCheirality,
      boost::optional<Pose3> body_P_sensor = boost::none)
      : Base(model, poseKey_a, poseKey_b, pointKey),
        measured_(measured),
        alpha_(alpha),
        K_(K),
        body_P_sensor_(body_P_sensor),
        throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  virtual ~ProjectionFactorRollingShutter() {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "ProjectionFactorRollingShutter, z = ";
    traits<Point2>::Print(measured_);
    std::cout << " rolling shutter interpolation param = " << alpha_;
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && (alpha_ == e->alpha()) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol) &&
           this->K_->equals(*e->K_, tol) &&
           (this->throwCheirality_ == e->throwCheirality_) &&
           (this->verboseCheirality_ == e->verboseCheirality_) &&
           ((!body_P_sensor_ && !e->body_P_sensor_) ||
            (body_P_sensor_ && e->body_P_sensor_ &&
             body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(
      const Pose3& pose_a, const Pose3& pose_b, const Point3& point,
      OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3) const override;

  /** return the measurement */
  const Point2& measured() const { return measured_; }

  /** return the calibration object */
  inline const boost::shared_ptr<Cal3_S2> calibration() const { return K_; }

  /** returns the rolling shutter interp param*/
  inline double alpha() const { return alpha_; }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measured_);
    ar& BOOST_SERIALIZATION_NVP(alpha_);
    ar& BOOST_SERIALIZATION_NVP(K_);
    ar& BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar& BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar& BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// traits
template <>
struct traits<ProjectionFactorRollingShutter>
    : public Testable<ProjectionFactorRollingShutter> {};

}  // namespace gtsam
