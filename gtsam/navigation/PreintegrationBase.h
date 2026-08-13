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

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>

#include <iosfwd>
#include <string>
#include <utility>

namespace gtsam {

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class GTSAM_EXPORT PreintegrationBase {
 public:
  typedef imuBias::ConstantBias Bias;
  typedef PreintegrationParams Params;

 protected:
  std::shared_ptr<Params> p_;

  /// Acceleration and gyro bias used for preintegration
  Bias biasHat_;

  /// Time interval from i to j
  double deltaTij_;

  /// Default constructor for serialization
  PreintegrationBase() {}

  /// Virtual destructor for serialization
  virtual ~PreintegrationBase() {}

 public:
  /// @name Constructors
  /// @{

  /**
   *  Constructor, initializes the variables in the base class
   *  @param p    Parameters, typically fixed in a single application
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  PreintegrationBase(const std::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias());

  /// @}

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements
  virtual void resetIntegration() = 0;

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements and set new bias
  void resetIntegrationAndSetBias(const Bias& biasHat);

  /// check parameters equality: checks whether shared pointer points to same Params object.
  bool matchesParamsWith(const PreintegrationBase& other) const {
    return p_.get() == other.p_.get();
  }

  /// shared pointer to params
  const std::shared_ptr<Params>& params() const {
    return p_;
  }

  /// const reference to params
  Params& p() const {
    return *p_;
  }

  /// @}

  /// @name Instance variables access
  /// @{
  const imuBias::ConstantBias& biasHat() const { return biasHat_; }
  double deltaTij() const { return deltaTij_; }

  virtual Vector3  deltaPij() const = 0;
  virtual Vector3  deltaVij() const = 0;
  virtual Rot3     deltaRij() const = 0;
  virtual NavState deltaXij() const = 0;

  /** Return the SO(3) tangent vector at local time t in [0, deltaTij]. */
  virtual Vector3 so3TangentAt(double t) const;

  /**
   * Deskew endpoint-exclusive ordered point batches over the integration
   * interval. Each column is a batch containing one or more stacked 3-vectors,
   * so points must have shape 3m x n.
   */
  Matrix deskewPoints(
      ConstMatrixView points,
      const Vector3& velocity_i = Vector3::Zero()) const;

  /** Deskew stacked point batches at explicit times and initial velocity. */
  Matrix deskewPointsAtTimes(
      ConstMatrixView points, const Vector& times,
      const Vector3& velocity_i = Vector3::Zero()) const;

  // Exposed for MATLAB
  Vector6 biasHatVector() const { return biasHat_.vector(); }
  /// @}

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const PreintegrationBase& pim);
  virtual void print(const std::string& s="") const;
  /// @}

  /// @name Main functionality
  /// @{

  /**
   * Subtract estimate and correct for sensor pose
   * Compute the derivatives due to non-identity body_P_sensor (rotation and centrifugal acc)
   * Ignore D_correctedOmega_measuredAcc as it is trivially zero
   */
  std::pair<Vector3, Vector3> correctMeasurementsBySensorPose(
      const Vector3& unbiasedAcc, const Vector3& unbiasedOmega,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc = {},
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega = {},
      OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega = {}) const;

  /**
   *  Update preintegrated measurements and get derivatives
   * It takes measured quantities in the j frame
   * Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
   */
  virtual void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
      const double dt, Matrix9* A, Matrix93* B, Matrix93* C) = 0;

  /// Version without derivatives
  virtual void integrateMeasurement(const Vector3& measuredAcc,
      const Vector3& measuredOmega, const double dt);

  /// Given the estimate of the bias, return a NavState tangent vector
  /// summarizing the preintegrated IMU measurements so far
  virtual Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 6> H = {}) const = 0;

  /**
   * Predict state at time j, for a given gravity vector in the nav frame.
   * This overload allows gravity to differ from params (eg. when gravity is
   * an optimized variable, see ImuFactorWithGravityDirection and
   * ImuFactorWithGravityVector); H3 is the Jacobian wrt that vector.
   */
  NavState predict(const NavState& state_i, const imuBias::ConstantBias& bias_i,
                   const Vector3& n_gravity,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 6> H2 = {},
                   OptionalJacobian<9, 3> H3 = {}) const;

  /// Predict state at time j, using the gravity vector from params
  NavState predict(const NavState& state_i, const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 6> H2 = {}) const;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
  }
#endif
};

namespace internal {

/** Calculate the 9-dof preintegration error for an explicit gravity vector. */
template <class PIM>
Vector9 preintegrationError(
    const PIM& pim, const NavState& state_i, const NavState& state_j,
    const imuBias::ConstantBias& bias_i, const Vector3& n_gravity,
    OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 = {},
    OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {}) {
  Matrix9 D_predict_state_i;
  Matrix96 D_predict_bias_i;
  Matrix93 D_predict_gravity;
  const NavState predictedState_j = pim.predict(
      state_i, bias_i, n_gravity, H1 ? &D_predict_state_i : nullptr,
      H3 ? &D_predict_bias_i : nullptr,
      H4 ? &D_predict_gravity : nullptr);

  Matrix9 D_error_state_j, D_error_predict;
  const Vector9 error = state_j.localCoordinates(
      predictedState_j, H2 ? &D_error_state_j : nullptr,
      H1 || H3 || H4 ? &D_error_predict : nullptr);

  if (H1) *H1 = D_error_predict * D_predict_state_i;
  if (H2) *H2 = D_error_state_j;
  if (H3) *H3 = D_error_predict * D_predict_bias_i;
  if (H4) *H4 = D_error_predict * D_predict_gravity;
  return error;
}

/** Calculate the 9-dof error using the gravity vector stored in params. */
template <class PIM>
Vector9 preintegrationError(
    const PIM& pim, const NavState& state_i, const NavState& state_j,
    const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 = {},
    OptionalJacobian<9, 6> H3 = {}) {
  return preintegrationError(pim, state_i, state_j, bias_i,
                             pim.params()->n_gravity, H1, H2, H3);
}

/** Assemble pose/velocity Jacobians for an explicit gravity vector. */
template <class PIM>
Vector9 preintegrationErrorAndJacobians(
    const PIM& pim, const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, const Vector3& n_gravity,
    OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
    OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {},
    OptionalJacobian<9, 6> H5 = {}, OptionalJacobian<9, 3> H6 = {}) {
  const NavState state_i(pose_i, vel_i), state_j(pose_j, vel_j);

  Matrix9 D_error_state_i, D_error_state_j;
  const Vector9 error = preintegrationError(
      pim, state_i, state_j, bias_i, n_gravity,
      H1 || H2 ? &D_error_state_i : nullptr,
      H3 || H4 ? &D_error_state_j : nullptr, H5, H6);

  // Separate NavState derivatives. Independent velocity variables retract by
  // straight addition rather than the NavState semidirect-product update.
  if (H1) *H1 = D_error_state_i.leftCols<6>();
  if (H2) *H2 = D_error_state_i.rightCols<3>() * state_i.R().transpose();
  if (H3) *H3 = D_error_state_j.leftCols<6>();
  if (H4) *H4 = D_error_state_j.rightCols<3>() * state_j.R().transpose();
  return error;
}

/** Assemble pose/velocity Jacobians using gravity stored in params. */
template <class PIM>
Vector9 preintegrationErrorAndJacobians(
    const PIM& pim, const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
    OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {},
    OptionalJacobian<9, 6> H5 = {}) {
  return preintegrationErrorAndJacobians(
      pim, pose_i, vel_i, pose_j, vel_j, bias_i,
      pim.params()->n_gravity, H1, H2, H3, H4, H5);
}

/**
 * Adapter mapping a gravity parametrization GRAVITY to the nav-frame gravity
 * vector expected by PreintegrationBase, with the chain-rule Jacobian block.
 * Two parametrizations are provided:
 * - Unit3: an optimized direction scaled by a fixed, known magnitude
 * - Point3: a free vector entangling direction and magnitude
 * See ImuFactorWithGravityDirection and ImuFactorWithGravityVector.
 */
template <class GRAVITY>
struct GravityParametrization;

template <>
struct GravityParametrization<Unit3> {
  constexpr static int dimension = 2;
  constexpr static bool usesMagnitude = true;
  static Vector3 vector(const Unit3& gravity, double magnitude,
                        OptionalJacobian<3, 2> H = {}) {
    return gravity.scaled(magnitude, H);
  }
};

template <>
struct GravityParametrization<Point3> {
  constexpr static int dimension = 3;
  constexpr static bool usesMagnitude = false;
  static Vector3 vector(const Point3& gravity, double /*magnitude*/,
                        OptionalJacobian<3, 3> H = {}) {
    if (H) H->setIdentity();
    return gravity;
  }
};

}  // namespace internal

}  /// namespace gtsam
