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
    boost::optional<Pose3> body_P_sensor; ///< The pose of the sensor in the body frame

    Params() :
        gyroscopeCovariance(I_3x3) {
    }

    virtual void print(const std::string& s) const;

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
  double deltaTij_; ///< Time interval from i to j
  Rot3 deltaRij_; ///< Preintegrated relative orientation (in frame i)
  Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

  /// Parameters
  boost::shared_ptr<Params> p_;

  /// Default constructor for serialization
  PreintegratedRotation() {
  }

public:
  /// Default constructor, resets integration to zero
  explicit PreintegratedRotation(const boost::shared_ptr<Params>& p) :
      p_(p) {
    resetIntegration();
  }

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration();

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

  void print(const std::string& s) const;
  bool equals(const PreintegratedRotation& other, double tol) const;

  /// @}

  /// Take the gyro measurement, correct it using the (constant) bias estimate
  /// and possibly the sensor pose, and then integrate it forward in time to yield
  /// an incremental rotation.
  Rot3 incrementalRotation(const Vector3& measuredOmega, const Vector3& biasHat,
      double deltaT, OptionalJacobian<3, 3> D_incrR_integratedOmega) const;

  /// Calculate an incremental rotation given the gyro measurement and a time interval,
  /// and update both deltaTij_ and deltaRij_.
  void integrateMeasurement(const Vector3& measuredOmega,
      const Vector3& biasHat, double deltaT, Matrix3* D_incrR_integratedOmega,
      Matrix3* F);

  /// Return a bias corrected version of the integrated rotation, with optional Jacobian
  Rot3 biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
      OptionalJacobian<3, 3> H = boost::none) const;

  /// Integrate coriolis correction in body frame rot_i
  Vector3 integrateCoriolis(const Rot3& rot_i) const;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) { // NOLINT
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & BOOST_SERIALIZATION_NVP(deltaRij_);
    ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
  }
};

} // namespace gtsam
