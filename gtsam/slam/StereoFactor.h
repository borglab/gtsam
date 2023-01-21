/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StereoFactor.h
 * @brief   A non-linear factor for stereo measurements
 * @author  Alireza Fathi
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/StereoCamera.h>

namespace gtsam {

/**
 * A Generic Stereo Factor
 * @ingroup slam
 */
template<class POSE, class LANDMARK>
class GenericStereoFactor: public NoiseModelFactorN<POSE, LANDMARK> {
private:

  // Keep a copy of measurement and calibration for I/O
  StereoPoint2 measured_;                      ///< the measurement
  Cal3_S2Stereo::shared_ptr K_;                ///< shared pointer to calibration
  boost::optional<POSE> body_P_sensor_;        ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
  bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

public:

  // shorthand for base class type
  typedef NoiseModelFactorN<POSE, LANDMARK> Base;             ///< typedef for base class
  typedef GenericStereoFactor<POSE, LANDMARK> This;           ///< typedef for this class (with templates)
  typedef boost::shared_ptr<GenericStereoFactor> shared_ptr;  ///< typedef for shared pointer to this object
  typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /**
   * Default constructor
   */
  GenericStereoFactor() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)),
      throwCheirality_(false), verboseCheirality_(false) {}

  /**
   * Constructor
   * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
   * @param model is the noise model in on the measurement
   * @param poseKey the pose variable key
   * @param landmarkKey the landmark variable key
   * @param K the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  GenericStereoFactor(const StereoPoint2& measured, const SharedNoiseModel& model,
      Key poseKey, Key landmarkKey, const Cal3_S2Stereo::shared_ptr& K,
      boost::optional<POSE> body_P_sensor = boost::none) :
    Base(model, poseKey, landmarkKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
    throwCheirality_(false), verboseCheirality_(false) {}

  /**
   * Constructor with exception-handling flags
   * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
   * @param model is the noise model in on the measurement
   * @param poseKey the pose variable key
   * @param landmarkKey the landmark variable key
   * @param K the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are rethrown
   * @param verboseCheirality determines whether exceptions are printed for Cheirality
   * @param body_P_sensor is the transform from body to sensor frame  (default identity)
   */
  GenericStereoFactor(const StereoPoint2& measured, const SharedNoiseModel& model,
      Key poseKey, Key landmarkKey, const Cal3_S2Stereo::shared_ptr& K,
      bool throwCheirality, bool verboseCheirality,
      boost::optional<POSE> body_P_sensor = boost::none) :
    Base(model, poseKey, landmarkKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
    throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  ~GenericStereoFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    measured_.print(s + ".z");
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const GenericStereoFactor* e = dynamic_cast<const GenericStereoFactor*> (&f);
    return e
        && Base::equals(f)
        && measured_.equals(e->measured_, tol)
        && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose, const Point3& point,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    try {
      if(body_P_sensor_) {
        if(H1) {
          gtsam::Matrix H0;
          StereoCamera stereoCam(pose.compose(*body_P_sensor_, H0), K_);
          StereoPoint2 reprojectionError(stereoCam.project(point, H1, H2) - measured_);
          *H1 = *H1 * H0;
          return reprojectionError.vector();
        } else {
          StereoCamera stereoCam(pose.compose(*body_P_sensor_), K_);
          return (stereoCam.project(point, H1, H2) - measured_).vector();
        }
      } else {
        StereoCamera stereoCam(pose, K_);
        return (stereoCam.project(point, H1, H2) - measured_).vector();
      }
    } catch(StereoCheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(3,6);
      if (H2) *H2 = Z_3x3;
      if (verboseCheirality_)
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
          " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
      if (throwCheirality_)
        throw StereoCheiralityException(this->key2());
    }
    return Vector3::Constant(2.0 * K_->fx());
  }

  /** return the measured */
  const StereoPoint2& measured() const {
    return measured_;
  }

  /** return the calibration object */
  inline const Cal3_S2Stereo::shared_ptr calibration() const {
    return K_;
  }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};

/// traits
template<class T1, class T2>
struct traits<GenericStereoFactor<T1, T2> > : public Testable<GenericStereoFactor<T1, T2> > {};

} // \ namespace gtsam
