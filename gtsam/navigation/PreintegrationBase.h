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

#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>

#include <iosfwd>

namespace gtsam {

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
/// @deprecated
struct PoseVelocityBias {
  Pose3 pose;
  Vector3 velocity;
  imuBias::ConstantBias bias;
  PoseVelocityBias(const Pose3& _pose, const Vector3& _velocity,
      const imuBias::ConstantBias _bias) :
      pose(_pose), velocity(_velocity), bias(_bias) {
  }
  PoseVelocityBias(const NavState& navState, const imuBias::ConstantBias _bias) :
      pose(navState.pose()), velocity(navState.velocity()), bias(_bias) {
  }
  NavState navState() const {
    return NavState(pose, velocity);
  }
};
#endif

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class PreintegrationBase {
 public:
  typedef imuBias::ConstantBias Bias;
  typedef PreintegrationParams Params;

 protected:

  /// Parameters. Declared mutable only for deprecated predict method.
  /// TODO(frank): make const once deprecated method is removed
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  mutable
#endif
  boost::shared_ptr<Params> p_;

  /// Acceleration and gyro bias used for preintegration
  Bias biasHat_;

  /// Time interval from i to j
  double deltaTij_;

  /**
   * Preintegrated navigation state, from frame i to frame j
   * Order is: theta, position, velocity
   * Note: relative position does not take into account velocity at time i, see deltap+, in [2]
   * Note: velocity is now also in frame i, as opposed to deltaVij in [2]
   */
#ifdef GTSAM_IMU_MANIFOLD_INTEGRATION
  Vector9 preintegrated_;
  Matrix93 preintegrated_H_biasAcc_;    ///< Jacobian of preintegrated preintegrated w.r.t. acceleration bias
  Matrix93 preintegrated_H_biasOmega_;  ///< Jacobian of preintegrated preintegrated w.r.t. angular rate bias
#else
  NavState deltaXij_;
  Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
  Matrix3 delPdelBiasAcc_;   ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_; ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;   ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
#endif

  /// Default constructor for serialization
  PreintegrationBase() {
    resetIntegration();
  }

public:
  /// @name Constructors
  /// @{

  /// @name Constructors
  /// @{

  /**
   *  Constructor, initializes the variables in the base class
   *  @param p    Parameters, typically fixed in a single application
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  PreintegrationBase(const boost::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias());

  /// @}

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements
  void resetIntegration();

  /// check parameters equality: checks whether shared pointer points to same Params object.
  bool matchesParamsWith(const PreintegrationBase& other) const {
    return p_.get() == other.p_.get();
  }

  /// shared pointer to params
  const boost::shared_ptr<Params>& params() const {
    return p_;
  }

  /// const reference to params
  const Params& p() const {
    return *boost::static_pointer_cast<Params>(p_);
  }

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  void set_body_P_sensor(const Pose3& body_P_sensor) {
    p_->body_P_sensor = body_P_sensor;
  }
#endif
/// @}

  /// @name Instance variables access
  /// @{
  const imuBias::ConstantBias& biasHat() const { return biasHat_; }
  double deltaTij() const { return deltaTij_; }

#ifdef GTSAM_IMU_MANIFOLD_INTEGRATION
  const Vector9& preintegrated() const { return preintegrated_; }
  Vector3 theta() const     { return preintegrated_.head<3>(); }
  Vector3 deltaPij() const  { return preintegrated_.segment<3>(3); }
  Vector3 deltaVij() const  { return preintegrated_.tail<3>(); }
  Rot3 deltaRij() const     { return Rot3::Expmap(theta()); }
  NavState deltaXij() const { return NavState::Retract(preintegrated_); }

  const Matrix93& preintegrated_H_biasAcc() const { return preintegrated_H_biasAcc_; }
  const Matrix93& preintegrated_H_biasOmega() const { return preintegrated_H_biasOmega_; }
#else
  const NavState& deltaXij() const { return deltaXij_; }
  const Rot3& deltaRij() const { return deltaXij_.attitude(); }
  Vector3 deltaPij() const     { return deltaXij_.position().vector(); }
  Vector3 deltaVij() const     { return deltaXij_.velocity(); }
#endif

  // Exposed for MATLAB
  Vector6 biasHatVector() const { return biasHat_.vector(); }
  /// @}

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const PreintegrationBase& pim);
  void print(const std::string& s) const;
  bool equals(const PreintegrationBase& other, double tol) const;
  /// @}

  /// @name Main functionality
  /// @{

  /// Subtract estimate and correct for sensor pose
  /// Compute the derivatives due to non-identity body_P_sensor (rotation and centrifugal acc)
  /// Ignore D_correctedOmega_measuredAcc as it is trivially zero
  std::pair<Vector3, Vector3> correctMeasurementsBySensorPose(
      const Vector3& unbiasedAcc, const Vector3& unbiasedOmega,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc = boost::none,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega = boost::none,
      OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega = boost::none) const;

#ifdef GTSAM_IMU_MANIFOLD_INTEGRATION

  // Update integrated vector on tangent manifold preintegrated with acceleration
  // Static, functional version.
  static Vector9 UpdatePreintegrated(const Vector3& a_body,
                                     const Vector3& w_body, const double dt,
                                     const Vector9& preintegrated,
                                     OptionalJacobian<9, 9> A = boost::none,
                                     OptionalJacobian<9, 3> B = boost::none,
                                     OptionalJacobian<9, 3> C = boost::none);

  // Version without derivatives
  void integrateMeasurement(const Vector3& measuredAcc, const Vector3& measuredOmega, const double dt);

#endif

  /// Update preintegrated measurements and get derivatives
  /// It takes measured quantities in the j frame
  /// Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
  /// NOTE(frank): implementation is different in two versions
  void integrateMeasurement(const Vector3& measuredAcc, const Vector3& measuredOmega, const double dt,
                            Matrix9* A, Matrix93* B, Matrix93* C);

  /// Given the estimate of the bias, return a NavState tangent vector
  /// summarizing the preintegrated IMU measurements so far
  /// NOTE(frank): implementation is different in two versions
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 6> H = boost::none) const;

  /// Predict state at time j
  NavState predict(const NavState& state_i, const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = boost::none,
                   OptionalJacobian<9, 6> H2 = boost::none) const;

  /// Calculate error given navStates
  Vector9 computeError(const NavState& state_i, const NavState& state_j,
                       const imuBias::ConstantBias& bias_i,
                       OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
                       OptionalJacobian<9, 6> H3) const;

  /// Compute errors w.r.t. preintegrated measurements and jacobians wrt pose_i, vel_i, bias_i, pose_j, bias_j
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H1 =
          boost::none, OptionalJacobian<9, 3> H2 = boost::none,
      OptionalJacobian<9, 6> H3 = boost::none, OptionalJacobian<9, 3> H4 =
          boost::none, OptionalJacobian<9, 6> H5 = boost::none) const;

#ifdef GTSAM_IMU_MANIFOLD_INTEGRATION
  // Compose the two pre-integrated 9D-vectors zeta01 and zeta02, with derivatives
  static Vector9 Compose(const Vector9& zeta01, const Vector9& zeta12,
                         double deltaT12,
                         OptionalJacobian<9, 9> H1 = boost::none,
                         OptionalJacobian<9, 9> H2 = boost::none);

  /// Merge in a different set of measurements and update bias derivatives accordingly
  /// The derivatives apply to the preintegrated Vector9
  void mergeWith(const PreintegrationBase& pim, Matrix9* H1, Matrix9* H2);
  /// @}
#endif

  /** Dummy clone for MATLAB */
  virtual boost::shared_ptr<PreintegrationBase> clone() const {
    return boost::shared_ptr<PreintegrationBase>();
  }

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// @name Deprecated
  /// @{

  /// @deprecated predict
  PoseVelocityBias predict(const Pose3& pose_i, const Vector3& vel_i,
      const imuBias::ConstantBias& bias_i, const Vector3& n_gravity,
      const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis = false) const;

  /// @}
#endif

  /// @}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_NVP(p_);
#ifdef GTSAM_IMU_MANIFOLD_INTEGRATION
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & bs::make_nvp("preintegrated_", bs::make_array(preintegrated_.data(), preintegrated_.size()));
    ar & bs::make_nvp("preintegrated_H_biasAcc_", bs::make_array(preintegrated_H_biasAcc_.data(), preintegrated_H_biasAcc_.size()));
    ar & bs::make_nvp("preintegrated_H_biasOmega_", bs::make_array(preintegrated_H_biasOmega_.data(), preintegrated_H_biasOmega_.size()));
#else
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & BOOST_SERIALIZATION_NVP(deltaXij_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & bs::make_nvp("delRdelBiasOmega_", bs::make_array(delRdelBiasOmega_.data(), delRdelBiasOmega_.size()));
    ar & bs::make_nvp("delPdelBiasAcc_", bs::make_array(delPdelBiasAcc_.data(), delPdelBiasAcc_.size()));
    ar & bs::make_nvp("delPdelBiasOmega_", bs::make_array(delPdelBiasOmega_.data(), delPdelBiasOmega_.size()));
    ar & bs::make_nvp("delVdelBiasAcc_", bs::make_array(delVdelBiasAcc_.data(), delVdelBiasAcc_.size()));
    ar & bs::make_nvp("delVdelBiasOmega_", bs::make_array(delVdelBiasOmega_.data(), delVdelBiasOmega_.size()));
#endif
  }
};

} /// namespace gtsam
