/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegratedRotation.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

/**
 * PreintegratedRotation is the base class for all PreintegratedMeasurements
 * classes (in AHRSFactor, ImuFactor, and CombinedImuFactor).
 * It includes the definitions of the preintegrated rotation.
 */
class PreintegratedRotation {
 public:

  /// Parameters for pre-integration:
  /// Usage: Create just a single Params and pass a shared pointer to the constructor
  struct Params {
    Matrix3 gyroscopeCovariance; ///< continuous-time "Covariance" of gyroscope measurements
    boost::optional<Vector3> omegaCoriolis; ///< Coriolis constant
    boost::optional<Pose3> body_P_sensor;  ///< The pose of the sensor in the body frame

    Params():gyroscopeCovariance(I_3x3) {}

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(gyroscopeCovariance);
      ar & BOOST_SERIALIZATION_NVP(omegaCoriolis);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor);
    }
  };

 protected:
  double deltaTij_;  ///< Time interval from i to j
  Rot3 deltaRij_;    ///< Preintegrated relative orientation (in frame i)
  Matrix3 delRdelBiasOmega_;  ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

  /// Parameters
  boost::shared_ptr<Params> p_;

  /// Default constructor for serialization
  PreintegratedRotation() {}

 public:
  /// Default constructor, resets integration to zero
  explicit PreintegratedRotation(const boost::shared_ptr<Params>& p) : p_(p) {
    resetIntegration();
  }

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration() {
    deltaTij_ = 0.0;
    deltaRij_ = Rot3();
    delRdelBiasOmega_ = Z_3x3;
  }

  /// @name Access instance variables
  /// @{
  const double& deltaTij() const {
    return deltaTij_;
  }
  const Rot3& deltaRij() const {
    return deltaRij_;
  }
  const Matrix3& delRdelBiasOmega() const {
    return delRdelBiasOmega_;
  }
  /// @}

  /// @name Testable
  /// @{

  void print(const std::string& s) const {
    std::cout << s << std::endl;
    std::cout << "    deltaTij [" << deltaTij_ << "]" << std::endl;
    std::cout << "    deltaRij.ypr = (" << deltaRij_.ypr().transpose() << ")" << std::endl;
  }

  bool equals(const PreintegratedRotation& other, double tol) const {
    return deltaRij_.equals(other.deltaRij_, tol) &&
           fabs(deltaTij_ - other.deltaTij_) < tol &&
           equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol);
  }

  /// @}

  /// Update preintegrated measurements
  void updateIntegratedRotationAndDeltaT(const Rot3& incrR, const double deltaT,
      OptionalJacobian<3, 3> H = boost::none) {
    deltaRij_ = deltaRij_.compose(incrR, H, boost::none);
    deltaTij_ += deltaT;
  }

  /**
   *  Update Jacobians to be used during preintegration
   *  TODO: explain arguments
   */
  void update_delRdelBiasOmega(const Matrix3& D_Rincr_integratedOmega,
      const Rot3& incrR, double deltaT) {
    const Matrix3 incrRt = incrR.transpose();
    delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - D_Rincr_integratedOmega * deltaT;
  }

  /// Return a bias corrected version of the integrated rotation, with optional Jacobian
  Rot3 biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
      OptionalJacobian<3, 3> H = boost::none) const {
    const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasOmegaIncr;
    const Rot3 deltaRij_biascorrected = deltaRij_.expmap(biasInducedOmega, boost::none, H);
    if (H) (*H) *= delRdelBiasOmega_;
    return deltaRij_biascorrected;
  }

  /// Integrate coriolis correction in body frame rot_i
  Vector3 integrateCoriolis(const Rot3& rot_i) const {
    if (!p_->omegaCoriolis) return Vector3::Zero();
    return rot_i.transpose() * (*p_->omegaCoriolis) * deltaTij_;
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {  // NOLINT
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & BOOST_SERIALIZATION_NVP(deltaRij_);
    ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
  }
};

}  // namespace gtsam
