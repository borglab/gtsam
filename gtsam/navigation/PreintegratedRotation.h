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

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegratedRotationParams {
  /// Continuous-time "Covariance" of gyroscope measurements
  /// The units for stddev are σ = rad/s/√Hz
  Matrix3 gyroscopeCovariance;
  boost::optional<Vector3> omegaCoriolis;  ///< Coriolis constant
  boost::optional<Pose3> body_P_sensor;    ///< The pose of the sensor in the body frame

  PreintegratedRotationParams() : gyroscopeCovariance(I_3x3) {}

  PreintegratedRotationParams(const Matrix3& gyroscope_covariance,
                              boost::optional<Vector3> omega_coriolis)
    : gyroscopeCovariance(gyroscope_covariance) {
      if (omega_coriolis)
        omegaCoriolis.reset(omega_coriolis.get());
  }

  virtual ~PreintegratedRotationParams() {}

  virtual void print(const std::string& s) const;
  virtual bool equals(const PreintegratedRotationParams& other, double tol=1e-9) const;

  void setGyroscopeCovariance(const Matrix3& cov)   { gyroscopeCovariance = cov;  }
  void setOmegaCoriolis(const Vector3& omega)       { omegaCoriolis.reset(omega); }
  void setBodyPSensor(const Pose3& pose)            { body_P_sensor.reset(pose);  }

  const Matrix3& getGyroscopeCovariance()     const { return gyroscopeCovariance; }
  boost::optional<Vector3> getOmegaCoriolis() const { return omegaCoriolis; }
  boost::optional<Pose3>   getBodyPSensor()   const { return body_P_sensor; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_NVP(gyroscopeCovariance);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor);

    // Provide support for Eigen::Matrix in boost::optional
    bool omegaCoriolisFlag = omegaCoriolis.is_initialized();
    ar & boost::serialization::make_nvp("omegaCoriolisFlag", omegaCoriolisFlag);
    if (omegaCoriolisFlag) {
      ar & BOOST_SERIALIZATION_NVP(*omegaCoriolis);
    }
  }

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
  boost::shared_ptr<Params> p_;

  double deltaTij_;           ///< Time interval from i to j
  Rot3 deltaRij_;             ///< Preintegrated relative orientation (in frame i)
  Matrix3 delRdelBiasOmega_;  ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

  /// Default constructor for serialization
  PreintegratedRotation() {}

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, resets integration to zero
  explicit PreintegratedRotation(const boost::shared_ptr<Params>& p) : p_(p) {
    resetIntegration();
  }

  /// Explicit initialization of all class members
  PreintegratedRotation(const boost::shared_ptr<Params>& p,
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
  const boost::shared_ptr<Params>& params() const {
    return p_;
  }
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
  void print(const std::string& s) const;
  bool equals(const PreintegratedRotation& other, double tol) const;
  /// @}

  /// @name Main functionality
  /// @{

  /// Take the gyro measurement, correct it using the (constant) bias estimate
  /// and possibly the sensor pose, and then integrate it forward in time to yield
  /// an incremental rotation.
  Rot3 incrementalRotation(const Vector3& measuredOmega, const Vector3& biasHat, double deltaT,
                           OptionalJacobian<3, 3> D_incrR_integratedOmega) const;

  /// Calculate an incremental rotation given the gyro measurement and a time interval,
  /// and update both deltaTij_ and deltaRij_.
  void integrateMeasurement(const Vector3& measuredOmega, const Vector3& biasHat, double deltaT,
                            OptionalJacobian<3, 3> D_incrR_integratedOmega = boost::none,
                            OptionalJacobian<3, 3> F = boost::none);

  /// Return a bias corrected version of the integrated rotation, with optional Jacobian
  Rot3 biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
                             OptionalJacobian<3, 3> H = boost::none) const;

  /// Integrate coriolis correction in body frame rot_i
  Vector3 integrateCoriolis(const Rot3& rot_i) const;

  /// @}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {  // NOLINT
    ar& BOOST_SERIALIZATION_NVP(p_);
    ar& BOOST_SERIALIZATION_NVP(deltaTij_);
    ar& BOOST_SERIALIZATION_NVP(deltaRij_);
    ar& BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
  }

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
  public:
	  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <>
struct traits<PreintegratedRotation> : public Testable<PreintegratedRotation> {};

}  /// namespace gtsam
