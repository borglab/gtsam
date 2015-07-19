/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationBase.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

/// Velocity in 3D is just a Vector3
typedef Vector3 Velocity3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 */
class NavState: private ProductLieGroup<Pose3, Velocity3> {
protected:
  typedef ProductLieGroup<Pose3, Velocity3> Base;
  typedef OptionalJacobian<9, 9> ChartJacobian;

public:
  // constructors
  NavState() {}
  NavState(const Pose3& pose, const Velocity3& vel) : Base(pose, vel) {}

  // access
  const Pose3& pose() const { return first; }
  const Point3& translation() const { return pose().translation(); }
  const Rot3& rotation() const { return pose().rotation(); }
  const Velocity3& velocity() const { return second; }
};

/// @deprecated
struct PoseVelocityBias {
  Pose3 pose;
  Vector3 velocity;
  imuBias::ConstantBias bias;
  PoseVelocityBias(const Pose3& _pose, const Vector3& _velocity, const imuBias::ConstantBias _bias)
      : pose(_pose), velocity(_velocity), bias(_bias) {}
  PoseVelocityBias(const NavState& navState, const imuBias::ConstantBias _bias)
      : pose(navState.pose()), velocity(navState.velocity()), bias(_bias) {}
  NavState navState() const { return NavState(pose,velocity);}
};

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class PreintegrationBase : public PreintegratedRotation {

 public:

  /// Parameters for pre-integration:
  /// Usage: Create just a single Params and pass a shared pointer to the constructor
  struct Params : PreintegratedRotation::Params {
    Matrix3 accelerometerCovariance;  ///< continuous-time "Covariance" of accelerometer
    Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty
    /// (to compensate errors in Euler integration)
    bool use2ndOrderIntegration;  ///< Controls the order of integration
    ///  (if false: p(t+1) = p(t) + v(t) deltaT ; if true: p(t+1) = p(t) + v(t) deltaT + 0.5 * acc(t) deltaT^2)
    bool use2ndOrderCoriolis;     ///< Whether to use second order Coriolis integration
    Vector3 gravity;              ///< Gravity constant

    Params()
        : accelerometerCovariance(I_3x3),
          integrationCovariance(I_3x3),
          use2ndOrderIntegration(false),
          use2ndOrderCoriolis(false),
          gravity(0, 0, 9.8) {}

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation::Params);
      ar & BOOST_SERIALIZATION_NVP(accelerometerCovariance);
      ar & BOOST_SERIALIZATION_NVP(integrationCovariance);
      ar & BOOST_SERIALIZATION_NVP(use2ndOrderIntegration);
      ar & BOOST_SERIALIZATION_NVP(use2ndOrderCoriolis);
      ar & BOOST_SERIALIZATION_NVP(gravity);
    }
  };

 protected:

  /// Acceleration and gyro bias used for preintegration
  imuBias::ConstantBias biasHat_;

  Vector3 deltaPij_;  ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
  Vector3 deltaVij_;  ///< Preintegrated relative velocity (in global frame)

  Matrix3 delPdelBiasAcc_;    ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_;  ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;    ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_;  ///< Jacobian of preintegrated velocity w.r.t. angular rate bias

  /// Default constructor for serialization
  PreintegrationBase() {}

 public:

  /**
   *  Constructor, initializes the variables in the base class
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param p    Parameters, typically fixed in a single application
   */
  PreintegrationBase(const boost::shared_ptr<const Params>& p,
                     const imuBias::ConstantBias& biasHat)
      : PreintegratedRotation(p), biasHat_(biasHat) {
    resetIntegration();
  }

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration();

  const Params& p() const { return *boost::static_pointer_cast<const Params>(p_);}

  /// getters
  const imuBias::ConstantBias& biasHat() const { return biasHat_; }
  const Vector3& deltaPij() const { return deltaPij_; }
  const Vector3& deltaVij() const { return deltaVij_; }
  const Matrix3& delPdelBiasAcc() const { return delPdelBiasAcc_; }
  const Matrix3& delPdelBiasOmega() const { return delPdelBiasOmega_; }
  const Matrix3& delVdelBiasAcc() const { return delVdelBiasAcc_; }
  const Matrix3& delVdelBiasOmega() const { return delVdelBiasOmega_; }

  // Exposed for MATLAB
  Vector6 biasHatVector() const { return biasHat_.vector(); }

  /// print
  void print(const std::string& s) const;

  /// check equality
  bool equals(const PreintegrationBase& other, double tol) const;

  /// Update preintegrated measurements
  void updatePreintegratedMeasurements(const Vector3& correctedAcc, const Rot3& incrR,
                                       const double deltaT, OptionalJacobian<9, 9> F);

  /// Update Jacobians to be used during preintegration
  void updatePreintegratedJacobians(const Vector3& correctedAcc,
                                    const Matrix3& D_Rincr_integratedOmega, const Rot3& incrR,
                                    double deltaT);

  void correctMeasurementsByBiasAndSensorPose(const Vector3& measuredAcc,
                                              const Vector3& measuredOmega,
                                              Vector3* correctedAcc,
                                              Vector3* correctedOmega);

  Vector3 biascorrectedDeltaPij(const imuBias::ConstantBias& biasIncr) const {
    return deltaPij_ + delPdelBiasAcc_ * biasIncr.accelerometer()
        + delPdelBiasOmega_ * biasIncr.gyroscope();
  }

  Vector3 biascorrectedDeltaVij(const imuBias::ConstantBias& biasIncr) const {
    return deltaVij_ + delVdelBiasAcc_ * biasIncr.accelerometer()
        + delVdelBiasOmega_ * biasIncr.gyroscope();
  }

  /// Integrate coriolis correction in body frame state_i
  Vector9 integrateCoriolis(const NavState& state_i) const;

  /// Recombine the preintegration, gravity, and coriolis in a single NavState tangent vector
  Vector9 recombinedPrediction(const NavState& state_i,
      const Rot3& deltaRij_biascorrected, const Vector3& deltaPij_biascorrected,
      const Vector3& deltaVij_biascorrected) const;

  /// Predict state at time j, with bias-corrected quantities given
  NavState predict(const NavState& navState, const Rot3& deltaRij_biascorrected,
      const Vector3& deltaPij_biascorrected,
      const Vector3& deltaVij_biascorrected) const;

  /// Predict state at time j
  NavState predict(const NavState& state_i,
      const imuBias::ConstantBias& bias_i) const;

  /// Compute errors w.r.t. preintegrated measurements and jacobians wrt pose_i, vel_i, bias_i, pose_j, bias_j
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j,
                                   const Vector3& vel_j, const imuBias::ConstantBias& bias_i,
                                   OptionalJacobian<9, 6> H1 = boost::none,
                                   OptionalJacobian<9, 3> H2 = boost::none,
                                   OptionalJacobian<9, 6> H3 = boost::none,
                                   OptionalJacobian<9, 3> H4 = boost::none,
                                   OptionalJacobian<9, 6> H5 = boost::none) const;

  /// @deprecated predict
  PoseVelocityBias predict(const Pose3& pose_i, const Vector3& vel_i,
      const imuBias::ConstantBias& bias_i, const Vector3& gravity, const Vector3& omegaCoriolis,
      const bool use2ndOrderCoriolis = false);

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaPij_);
    ar & BOOST_SERIALIZATION_NVP(deltaVij_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasOmega_);
  }
};

}  /// namespace gtsam
