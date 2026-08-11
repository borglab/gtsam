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

  /// Calculate error given navStates and a gravity vector in the nav frame
  Vector9 computeError(const NavState& state_i, const NavState& state_j,
                       const imuBias::ConstantBias& bias_i,
                       const Vector3& n_gravity,
                       OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
                       OptionalJacobian<9, 6> H3,
                       OptionalJacobian<9, 3> H4) const;

  /// Calculate error given navStates, using the gravity vector from params
  Vector9 computeError(const NavState& state_i, const NavState& state_j,
                       const imuBias::ConstantBias& bias_i,
                       OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2,
                       OptionalJacobian<9, 6> H3) const;

  /**
   * Compute errors w.r.t. preintegrated measurements and jacobians
   * wrt pose_i, vel_i, bias_i, pose_j, bias_j and the given gravity vector
   */
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, const Vector3& n_gravity,
      OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
      OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {},
      OptionalJacobian<9, 6> H5 = {}, OptionalJacobian<9, 3> H6 = {}) const;

  /**
   * Compute errors w.r.t. preintegrated measurements and jacobians
   * wrt pose_i, vel_i, bias_i, pose_j, bias_j
   */
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 6> H1 = {}, OptionalJacobian<9, 3> H2 = {},
      OptionalJacobian<9, 6> H3 = {}, OptionalJacobian<9, 3> H4 = {},
      OptionalJacobian<9, 6> H5 = {}) const;

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
