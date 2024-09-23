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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/std_optional_serialization.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/dllexport.h"

namespace gtsam {

namespace internal {
/**
 * @brief Function object for incremental rotation.
 * @param measuredOmega The measured angular velocity (as given by the sensor)
 * @param deltaT The time interval over which the rotation is integrated.
 * @param body_P_sensor Optional transform between body and IMU.
 */
struct GTSAM_EXPORT IncrementalRotation {
  const Vector3& measuredOmega;
  const double deltaT;
  const std::optional<Pose3>& body_P_sensor;

  /**
   * @brief Integrate angular velocity, but corrected by bias.
   * @param bias The bias estimate
   * @param H_bias Jacobian of the rotation w.r.t. bias.
   * @return The incremental rotation
   */
  Rot3 operator()(const Vector3& bias,
                  OptionalJacobian<3, 3> H_bias = {}) const;
};

}  // namespace internal

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegratedRotationParams {
  /// Continuous-time "Covariance" of gyroscope measurements
  /// The units for stddev are σ = rad/s/√Hz
  Matrix3 gyroscopeCovariance;
  std::optional<Vector3> omegaCoriolis;  ///< Coriolis constant
  std::optional<Pose3> body_P_sensor;    ///< The pose of the sensor in the body frame

  PreintegratedRotationParams() : gyroscopeCovariance(I_3x3) {}

  PreintegratedRotationParams(const Matrix3& gyroscope_covariance,
                              std::optional<Vector3> omega_coriolis)
    : gyroscopeCovariance(gyroscope_covariance) {
      if (omega_coriolis) {
        omegaCoriolis = *omega_coriolis;
      }
  }

  virtual ~PreintegratedRotationParams() {}

  virtual void print(const std::string& s) const;
  virtual bool equals(const PreintegratedRotationParams& other, double tol=1e-9) const;

  void setGyroscopeCovariance(const Matrix3& cov)   { gyroscopeCovariance = cov;  }
  void setOmegaCoriolis(const Vector3& omega)       { omegaCoriolis = omega; }
  void setBodyPSensor(const Pose3& pose)            { body_P_sensor = pose;  }

  const Matrix3& getGyroscopeCovariance()     const { return gyroscopeCovariance; }
  std::optional<Vector3> getOmegaCoriolis() const { return omegaCoriolis; }
  std::optional<Pose3>   getBodyPSensor()   const { return body_P_sensor; }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(gyroscopeCovariance);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor);

    // Provide support for Eigen::Matrix in std::optional
    bool omegaCoriolisFlag = omegaCoriolis.has_value();
    ar & boost::serialization::make_nvp("omegaCoriolisFlag", omegaCoriolisFlag);
    if (omegaCoriolisFlag) {
      ar & BOOST_SERIALIZATION_NVP(*omegaCoriolis);
    }
  }
#endif

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
public:
	GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

/**
 * PreintegratedRotation is the base class for all PreintegratedMeasurements
 * classes (in AHRSFactor, ImuFactor, and CombinedImuFactor).
 * It includes the definitions of the preintegrated rotation.
 */
class GTSAM_EXPORT PreintegratedRotation {
 public:
  typedef PreintegratedRotationParams Params;

 protected:
  /// Parameters
  std::shared_ptr<Params> p_;

  double deltaTij_;           ///< Time interval from i to j
  Rot3 deltaRij_;             ///< Preintegrated relative orientation (in frame i)
  Matrix3 delRdelBiasOmega_;  ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

  /// Default constructor for serialization
  PreintegratedRotation() {}

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, resets integration to zero
  explicit PreintegratedRotation(const std::shared_ptr<Params>& p) : p_(p) {
    resetIntegration();
  }

  /// Explicit initialization of all class members
  PreintegratedRotation(const std::shared_ptr<Params>& p,
                        double deltaTij, const Rot3& deltaRij,
                        const Matrix3& delRdelBiasOmega)
      : p_(p), deltaTij_(deltaTij), deltaRij_(deltaRij), delRdelBiasOmega_(delRdelBiasOmega) {}

  /// @}

  /// @name Basic utilities
  /// @{

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration();

  /// check parameters equality: checks whether shared pointer points to same Params object.
  bool matchesParamsWith(const PreintegratedRotation& other) const {
    return p_ == other.p_;
  }
  /// @}

  /// @name Access instance variables
  /// @{
  const std::shared_ptr<Params>& params() const { return p_; }
  const double& deltaTij() const { return deltaTij_; }
  const Rot3& deltaRij() const { return deltaRij_; }
  const Matrix3& delRdelBiasOmega() const { return delRdelBiasOmega_; }
  /// @}

  /// @name Testable
  /// @{
  void print(const std::string& s) const;
  bool equals(const PreintegratedRotation& other, double tol) const;
  /// @}

  /// @name Main functionality
  /// @{

  /**
   * @brief Calculate an incremental rotation given the gyro measurement and a
   * time interval, and update both deltaTij_ and deltaRij_.
   * @param measuredOmega The measured angular velocity (as given by the sensor)
   * @param bias The biasHat estimate
   * @param deltaT The time interval
   * @param F optional Jacobian of internal compose, used in AhrsFactor.
   */
  void integrateGyroMeasurement(const Vector3& measuredOmega,
                                const Vector3& biasHat, double deltaT,
                                OptionalJacobian<3, 3> F = {});

  /**
   * @brief Return a bias corrected version of the integrated rotation.
   * @param biasOmegaIncr An increment with respect to biasHat used above.
   * @param H optional Jacobian of the correction w.r.t. the bias increment.
   * @note The *key* functionality of this class used in optimizing the bias.
   */
  Rot3 biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
                             OptionalJacobian<3, 3> H = {}) const;

  /// Integrate coriolis correction in body frame rot_i
  Vector3 integrateCoriolis(const Rot3& rot_i) const;

  /// @}

  /// @name Deprecated API
  /// @{

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @deprecated: use IncrementalRotation functor with sane Jacobian
  inline Rot3 GTSAM_DEPRECATED incrementalRotation(
      const Vector3& measuredOmega, const Vector3& bias, double deltaT,
      OptionalJacobian<3, 3> D_incrR_integratedOmega) const {
    internal::IncrementalRotation f{measuredOmega, deltaT, p_->body_P_sensor};
    Rot3 incrR = f(bias, D_incrR_integratedOmega);
    // Backwards compatible "weird" Jacobian, no longer used.
    if (D_incrR_integratedOmega) *D_incrR_integratedOmega /= -deltaT;
    return incrR;
  }

  /// @deprecated: use integrateGyroMeasurement from now on
  /// @note this returned hard-to-understand Jacobian D_incrR_integratedOmega.
  void GTSAM_DEPRECATED integrateMeasurement(
      const Vector3& measuredOmega, const Vector3& biasHat, double deltaT,
      OptionalJacobian<3, 3> D_incrR_integratedOmega, OptionalJacobian<3, 3> F);

#endif

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {  // NOLINT
    ar& BOOST_SERIALIZATION_NVP(p_);
    ar& BOOST_SERIALIZATION_NVP(deltaTij_);
    ar& BOOST_SERIALIZATION_NVP(deltaRij_);
    ar& BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
  }
#endif

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
  public:
	  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <>
struct traits<PreintegratedRotation> : public Testable<PreintegratedRotation> {};

}  /// namespace gtsam
