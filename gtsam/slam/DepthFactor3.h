
/**
 * @file DepthFactor3.h
 * @brief Depth Factor for 3D landmarks with depth measurements.
 * A factor that relates a camera pose, a 3D landmark, and a depth measurement
 * (u, v, depth) in pixel coordinates.
 * @author junlinp
 * @date Nov 9, 2025
 */

#pragma once

#include <optional>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/DepthCamera3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor representing a visual measurement with depth
 * @tparam POSE Camera pose type (typically Pose3)
 * @tparam LANDMARK Landmark type (typically Point3)
 */
template<class POSE, class LANDMARK>
class DepthFactor3: public NoiseModelFactorN<POSE, LANDMARK> {
protected:

  // Keep a copy of measurement and calibration for I/O
  Point3 measured_;                ///< 3D measurement (u, v, depth) in pixel coordinates
  std::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object
  std::optional<Pose3> body_P_sensor_;  ///< The pose of the sensor in the body frame

public:

  /// shorthand for base class type
  typedef NoiseModelFactorN<POSE, LANDMARK> Base;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for this class
  typedef DepthFactor3<POSE, LANDMARK> This;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor
  DepthFactor3() :
      measured_(0.0, 0.0, 0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  /**
   * Constructor
   * @param measured is the 3D measurement (u, v, depth) in pixel coordinates
   * @param model is the noise model
   * @param poseKey is the index of the camera pose
   * @param landmarkKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  DepthFactor3(const Point3& measured, const SharedNoiseModel& model,
      const Key poseKey, Key landmarkKey, const Cal3_S2::shared_ptr& K,
      const std::optional<Pose3>& body_P_sensor = {}) :
        Base(model, poseKey, landmarkKey), measured_(measured), K_(K),
        body_P_sensor_(body_P_sensor) {
          // compute the noise model for converted measurement.
          // jacobian of [u, v, depth] to [u * depth, v * depth, depth]
          double u = measured_.x();          
          double v = measured_.y();
          double depth = measured_.z();
          Matrix33 J = (Matrix33() << depth, 0, u,
                                0, depth, v,
                                0, 0, 1).finished();
          // sigmas() returns standard deviations, need to square them for covariance
          Vector3 sigmas = model->sigmas();
          Vector3 sigmas_sq = sigmas.array().square();
          Matrix33 original_cov = sigmas_sq.asDiagonal();
          Matrix33 new_cov = J * original_cov * J.transpose();
          Matrix33 intrinsics_inv = (Matrix33() << 1.0 / K_->fx(), 0, -K_->px() / K_->fx(),
                                0, 1.0 / K_->fy(), -K_->py() / K_->fy(),
                                0, 0, 1.0).finished();
          Matrix33 new_cov_intrinsics = intrinsics_inv * new_cov * intrinsics_inv.transpose();
          SharedNoiseModel new_model = noiseModel::Gaussian::Covariance(new_cov_intrinsics);
          // Directly assign the new noise model (noiseModel_ is protected in NoiseModelFactor)
          this->noiseModel_ = new_model;
        }

  /** Virtual destructor */
  ~DepthFactor3() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "DepthFactor3",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << "  measurement: " << measured_.transpose() << std::endl;
    if (K_) K_->print("  calibration: ");
    if (body_P_sensor_) {
      body_P_sensor_->print("  body_P_sensor: ");
    }
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && 
           traits<Point3>::Equals(this->measured_, e->measured_, tol) && 
           this->K_->equals(*e->K_, tol) &&
           ((!body_P_sensor_ && !e->body_P_sensor_) ||
            (body_P_sensor_ && e->body_P_sensor_ && 
             body_P_sensor_->equals(*e->body_P_sensor_, tol)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const POSE& pose, const LANDMARK& landmark,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    try {
      DepthCamera3<Cal3_S2> camera(measured_, K_, body_P_sensor_);
      return camera.project(pose, landmark, H1, H2);
    } catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(3, 6);
      if (H2) *H2 = Matrix::Zero(3, 3);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
          " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
      return Vector::Ones(3) * 2.0 * K_->fx();
    }
  }

  /** return the measurement */
  const Point3& measurement() const {
    return measured_;
  }

  /** return the calibration object */
  inline const Cal3_S2::shared_ptr calibration() const {
    return K_;
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
#endif
};
} // \ namespace gtsam

