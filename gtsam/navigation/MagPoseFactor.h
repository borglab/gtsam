/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to estimate rotation of a Pose2 or Pose3 given a magnetometer reading.
 * This version uses the measurement model bM = scale * bRn * direction + bias,
 * where bRn is the rotation of the body in the nav frame, and scale, direction,
 * and bias are assumed to be known. If the factor is constructed with a
 * body_P_sensor, then the magnetometer measurements and bias should be
 * expressed in the sensor frame.
 */
template <class POSE>
class MagPoseFactor: public NoiseModelFactorN<POSE> {
 private:
  using This = MagPoseFactor<POSE>;
  using Base = NoiseModelFactorN<POSE>;
  using Point = typename POSE::Translation; ///< Could be a Vector2 or Vector3 depending on POSE.
  using Rot = typename POSE::Rotation;

  const Point measured_; ///< The measured magnetometer data in the body frame.
  const Point nM_; ///< Local magnetic field (mag output units) in the nav frame.
  const Point bias_; ///< The bias vector (mag output units) in the body frame.
  boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame.

  static const int MeasDim = Point::RowsAtCompileTime;
  static const int PoseDim = traits<POSE>::dimension;
  static const int RotDim = traits<Rot>::dimension;

  /// Shorthand for a smart pointer to a factor.
  using shared_ptr = boost::shared_ptr<MagPoseFactor<POSE>>;

  /// Concept check by type.
  GTSAM_CONCEPT_TESTABLE_TYPE(POSE)
  GTSAM_CONCEPT_POSE_TYPE(POSE)

 public:
  using Base::evaluateError;

  ~MagPoseFactor() override {}

  /// Default constructor - only use for serialization.
  MagPoseFactor() {}

  /**
   * Construct the factor.
   * @param pose_key of the unknown pose nPb in the factor graph
   * @param measured magnetometer reading, a Point2 or Point3
   * @param scale by which a unit vector is scaled to yield a magnetometer reading
   * @param direction of the local magnetic field, see e.g. http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
   * @param bias of the magnetometer, modeled as purely additive (after scaling)
   * @param model of the additive Gaussian noise that is assumed
   * @param body_P_sensor an optional transform of the magnetometer in the body frame
   */
  MagPoseFactor(Key pose_key,
                const Point& measured,
                double scale,
                const Point& direction,
                const Point& bias,
                const SharedNoiseModel& model,
                const boost::optional<POSE>& body_P_sensor)
      : Base(model, pose_key),
        measured_(body_P_sensor ? body_P_sensor->rotation() * measured : measured),
        nM_(scale * direction.normalized()),
        bias_(body_P_sensor ? body_P_sensor->rotation() * bias : bias),
        body_P_sensor_(body_P_sensor) {}

  /// @return a deep copy of this factor.
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Implement functions needed for Testable.

  // Print out the factor.
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    gtsam::print(Vector(nM_), "local field (nM): ");
    gtsam::print(Vector(measured_), "measured field (bM): ");
    gtsam::print(Vector(bias_), "magnetometer bias: ");
  }

  /// Equals function.
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol) &&
        gtsam::equal_with_abs_tol(this->measured_, e->measured_, tol) &&
        gtsam::equal_with_abs_tol(this->nM_, e->nM_, tol) &&
        gtsam::equal_with_abs_tol(this->bias_, e->bias_, tol);
  }

  /// Implement functions needed to derive from Factor.

  /**
   * Return the factor's error h(x) - z, and the optional Jacobian. Note that
   * the measurement error is expressed in the body frame.
   */
  Vector evaluateError(const POSE& nPb, OptionalMatrixType H) const override {
    // Predict the measured magnetic field h(x) in the *body* frame.
    // If body_P_sensor was given, bias_ will have been rotated into the body frame.
    Matrix H_rot = Matrix::Zero(MeasDim, RotDim);
    const Point hx = nPb.rotation().unrotate(nM_, H_rot, OptionalNone) + bias_;

    if (H) {
      // Fill in the relevant part of the Jacobian (just rotation columns).
      *H = Matrix::Zero(MeasDim, PoseDim);
      const size_t rot_col0 = nPb.rotationInterval().first;
      (*H).block(0, rot_col0, MeasDim, RotDim) = H_rot;
    }

    return (hx - measured_);
  }

 private:
  /// Serialization function.
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(nM_);
    ar & BOOST_SERIALIZATION_NVP(bias_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};  // \class MagPoseFactor

} /// namespace gtsam
