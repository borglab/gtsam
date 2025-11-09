
/**
 * @file DepthFactor3.h
 * @brief Depth Factor for 3D landmarks with depth measurements.
 * A factor that relates a camera pose, a 3D landmark, and a depth measurement
 * (u, v, depth) in pixel coordinates.
 * @author junlinp
 * @date Nov 9, 2025
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/geometry/DepthCamera3.h>

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
   */
  DepthFactor3(const Point3& measured, const SharedNoiseModel& model,
      const Key poseKey, Key landmarkKey, const Cal3_S2::shared_ptr& K) :
        Base(model, poseKey, landmarkKey), measured_(measured), K_(K) {}

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
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && 
           traits<Point3>::Equals(this->measured_, e->measured_, tol) && 
           this->K_->equals(*e->K_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const POSE& pose, const LANDMARK& landmark,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    try {
      DepthCamera3<Cal3_S2> camera(measured_, K_);
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
  }
#endif
};
} // \ namespace gtsam

