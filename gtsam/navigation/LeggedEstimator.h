/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LeggedEstimator.h
 * @date February 2026
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LeftLinearEKF.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gtsam {

/// Body-frame contact measurement for one foot.
struct ContactMeasurement {
  size_t foot = 0;
  Vector3 bodyPoint = Vector3::Zero();
  /// True when this measurement corresponds to a new swing-to-stance touchdown.
  bool touchdown = false;
};

/// Common estimator parameters shared by all four variants.
struct LeggedEstimatorParams {
  std::shared_ptr<PreintegrationParams> preintegrationParams;
  Pose3 body_P_imu = Pose3(Rot3(), Point3(0.30, 0.0, 0.15));
  double footholdProcessSigma = 1e-4;
  double footholdInitSigma = 5e-1;
  Matrix3 contactCovariance =
      (Vector3(0.03 * 0.03, 0.03 * 0.03, 0.02 * 0.02)).asDiagonal();
  double heightPriorSigma = 0.15;
  /// Enable robust contact noise for graph-based variants.
  bool useRobustContactNoise = false;
  /// Scalar-Huber threshold for robust contact factors in graph-based variants.
  double robustContactHuberK = 2.0;
  /// Constant IMU bias removed from the raw gyroscope and accelerometer data.
  imuBias::ConstantBias imuBias;
  /// Accelerometer bias random-walk sigma used by the combined smoother.
  double biasAccRandomWalkSigma = 5e-3;
  /// Gyroscope bias random-walk sigma used by the combined smoother.
  double biasOmegaRandomWalkSigma = 1e-4;
  /// Run the one-time full-contact initializer when available.
  bool useFullContactInitialization = true;
  /// Replace a leaving foot by a fresh independent prior.
  bool marginalizeLeavingFoot = true;
};

/// Common runtime interface shared by all four legged estimator variants.
class GTSAM_EXPORT LeggedEstimator {
 public:
  /// Destroy the estimator interface.
  virtual ~LeggedEstimator() = default;

  /// Enable contact height priors at the supplied terrain height.
  void turnHeightPriorOn(double terrainHeight) {
    terrainHeight_ = terrainHeight;
  }

  /// Disable contact height priors.
  void turnHeightPriorOff() { terrainHeight_.reset(); }

  /// Predict the estimator forward with one IMU sample.
  virtual void predict(const Vector3& omegaBody,
                       const Vector3& specificForceBody, double dt) = 0;

  /// Process the currently active contacts at the current estimator time.
  virtual void processContacts(
      const std::vector<ContactMeasurement>& activeContacts) = 0;

  /**
   * Return the current propagated estimate as `ExtendedPose3d`.
   *
   * The state layout is `(R, p, v, f_1, ..., f_k)`, where the first two
   * translational blocks are base position and base velocity, followed by one
   * world-frame foothold block per foot.
   */
  virtual ExtendedPose3d estimate() const = 0;

  /// Return the IMU bias used or estimated by the current estimator.
  virtual imuBias::ConstantBias estimateBias() const = 0;

 protected:
  /// Optional terrain height used internally for contact height priors.
  const std::optional<double>& terrainHeight() const { return terrainHeight_; }

  /// Build the shared `ExtendedPose3d` state layout used by all estimators.
  static ExtendedPose3d MakeEstimate(const NavState& navState,
                                     const Matrix& footholds);

  /// Recover the base `NavState` from an `ExtendedPose3d` estimate.
  static NavState EstimateNavState(const ExtendedPose3d& estimate) {
    return NavState(estimate.rotation(), estimate.x(0), estimate.x(1));
  }

  /// Recover the foothold matrix from an `ExtendedPose3d` estimate.
  static Matrix EstimateFootholds(const ExtendedPose3d& estimate);

 private:
  std::optional<double> terrainHeight_;
};

/**
 * Invariant filter: LeftLinearEKF on ExtendedPose3(2+k).
 *
 * The state is a single dynamic `ExtendedPose3` containing
 * `(R, p, v, f_1, ..., f_k)` in `R,p,v` order. The prediction step is
 * left-linear, and the contact measurement model has Jacobians that are
 * invariant in the state coordinates.
 *
 * This is very similar to the Contact-aided invariant filter from
 *
 *   Hartley, Ross, Maani Ghaffari, Ryan M. Eustice, and Jessy W. Grizzle.
 *   "Contact-aided invariant extended Kalman filtering for robot state
 *   estimation." The International Journal of Robotics Research 39, no. 4
 *   (2020): 402-430.
 */
class GTSAM_EXPORT LeggedInvariantEKF : public LeftLinearEKF<ExtendedPose3d>,
                                        public LeggedEstimator {
 public:
  using EkfBase = LeftLinearEKF<ExtendedPose3d>;
  using TangentVector = typename EkfBase::TangentVector;
  using Jacobian = typename EkfBase::Jacobian;
  using Covariance = typename EkfBase::Covariance;

  /// Construct the ExtendedPose3-based EKF.
  LeggedInvariantEKF(const NavState& navState0, const Matrix& footholds0,
                     const Matrix& P0, const LeggedEstimatorParams& params,
                     const std::vector<std::string>& footNames = {});

  /// Return the current full covariance.
  Matrix covariance() const { return EkfBase::covariance(); }

  /// Number of feet tracked by the estimator.
  size_t numFeet() const { return numFeet_; }

  /// Foot names in state order.
  const std::vector<std::string>& footNames() const { return footNames_; }

  /// Shared estimator parameters.
  const LeggedEstimatorParams& params() const { return params_; }

  /// Current estimate in the shared `ExtendedPose3d` layout.
  ExtendedPose3d estimate() const override {
    return MakeEstimate(baseState(), footholdMatrix());
  }

  /// Return the fixed IMU bias used by the filter.
  imuBias::ConstantBias estimateBias() const override {
    return params_.imuBias;
  }

  /// Predict forward using one IMU sample.
  void predict(const Vector3& omegaBody, const Vector3& specificForceBody,
               double dt) override;

  /// Process all currently active contacts for the current step.
  void processContacts(
      const std::vector<ContactMeasurement>& activeContacts) override;

  /// Autonomous flow used by the left-linear prediction step.
  struct AutonomousFlow {
    /// Construct the autonomous-flow functor.
    explicit AutonomousFlow(size_t numFeet, double dt)
        : numFeet(numFeet), dt(dt) {}

    /// Return the differential of the flow at the identity.
    Matrix dIdentity() const {
      Matrix Phi = Matrix::Identity(9 + 3 * static_cast<int>(numFeet),
                                    9 + 3 * static_cast<int>(numFeet));
      Phi.block(3, 6, 3, 3) = I_3x3 * dt;
      return Phi;
    }

    /// Advance the position block while keeping velocity and footholds fixed.
    ExtendedPose3d operator()(const ExtendedPose3d& state) const {
      Matrix blocks = state.xMatrix();
      blocks.col(0) += blocks.col(1) * dt;
      return ExtendedPose3d(state.rotation(), blocks);
    }
    size_t numFeet;
    double dt;
  };

  /// Build an `ExtendedPose3(2+k)` state from base state and world footholds.
  static ExtendedPose3d MakeState(const NavState& navState,
                                  const Matrix& footholds);

  /// Build the gravity-only left increment for one prediction step.
  static ExtendedPose3d GravityIncrement(size_t numFeet, const Vector3& gravity,
                                         double dt);

  /// Build the IMU-only right increment for one prediction step.
  static ExtendedPose3d ImuIncrement(size_t numFeet, const Vector3& omegaBody,
                                     const Vector3& specificForceBody,
                                     double dt);

  /// Return the `ExtendedPose3` block index corresponding to a foot number.
  static size_t FootColumn(size_t foot) { return 2 + foot; }

 protected:
  NavState baseState() const {
    return {this->X_.rotation(), this->X_.x(0), this->X_.x(1)};
  }
  Matrix footholdMatrix() const {
    return this->X_.xMatrix().rightCols(static_cast<Eigen::Index>(numFeet()));
  }
  void resetFootToMeasurement(size_t foot, const Vector3& bodyPoint);
  void marginalizeFoot(size_t foot);
  virtual void applyContactUpdate(
      const std::vector<ContactMeasurement>& activeContacts);
  bool awaitingFullContactInitialization() const {
    return params_.useFullContactInitialization && !fullContactInitialized_;
  }

 private:
  bool maybeInitializeFromFullContact(
      const std::vector<ContactMeasurement>& activeContacts,
      const std::vector<bool>& activeFeet);
  Covariance processNoise(double dt) const;
  void applySingleContactUpdate(size_t foot, const Vector3& bodyPoint,
                                const Matrix3& covariance);
  void applySingleHeightPrior(size_t foot, double terrainHeight);

  size_t numFeet_;
  LeggedEstimatorParams params_;
  std::vector<std::string> footNames_;
  std::vector<bool> inContact_;
  std::vector<bool> initialized_;
  bool fullContactInitialized_ = false;
};

/**
 * Invariant filter with local graph fragment.
 *
 * This variant combines the two ideas introduced above: it uses the
 * `ExtendedPose3(2+k)` state of `LeggedInvariantEKF`, but performs the
 * measurement phase as a small nonlinear graph solve rather than as sequential
 * EKF corrections.
 */
class GTSAM_EXPORT LeggedInvariantIEKF : public LeggedInvariantEKF {
 public:
  /// Construct the graph-update ExtendedPose3 estimator.
  LeggedInvariantIEKF(const NavState& navState0, const Matrix& footholds0,
                      const Matrix& P0, const LeggedEstimatorParams& params,
                      const std::vector<std::string>& footNames = {});

 protected:
  void applyContactUpdate(
      const std::vector<ContactMeasurement>& activeContacts) override;
};

/**
 * Fixed-lag smoother over NavState and contact-episode footholds.
 *
 * The first two classes are single-step estimators that maintain one Gaussian
 * belief at the current time. This final variant instead builds a sliding
 * window over time using `BatchFixedLagSmoother`. Base states are created only
 * at contact events and are linked by preintegrated `ImuFactor2` motion
 * factors, while footholds are represented by contact-episode landmark
 * variables that are created on touchdown and naturally disappear once they
 * fall outside the lag window.
 *
 * This version is both a more realistic estimator for delayed multi-step
 * inference and an example of how the same measurement ideas can be expressed
 * in GTSAM as a smoother rather than as an EKF. It is deliberately kept in the
 * same file as the filter variants so users can read the progression from
 * sequential EKF, to local graph update, to fixed-lag smoothing in one place.
 */
class GTSAM_EXPORT LeggedFixedLagSmoother : public LeggedEstimator {
 public:
  /// Construct the fixed-lag smoother variant.
  LeggedFixedLagSmoother(const NavState& navState0, const Matrix& footholds0,
                         const Matrix9& baseCovariance0,
                         const LeggedEstimatorParams& params, double lagSeconds,
                         const std::vector<std::string>& footNames = {});

  /// Destroy the fixed-lag smoother variant.
  ~LeggedFixedLagSmoother() override;

  /// Number of feet tracked by the smoother front-end.
  size_t numFeet() const { return numFeet_; }

  /// Current estimate in the shared `ExtendedPose3d` layout.
  ExtendedPose3d estimate() const override;

  /// Return the current single shared IMU bias estimate.
  imuBias::ConstantBias estimateBias() const override { return biasEstimate_; }

  /// Accumulate one IMU sample and advance the dead-reckoned estimate.
  void predict(const Vector3& omegaBody, const Vector3& specificForceBody,
               double dt) override;

  /// Process the active contacts for the current step.
  void processContacts(
      const std::vector<ContactMeasurement>& activeContacts) override;

 private:
  bool maybeInitializeFromFullContact(
      const std::vector<ContactMeasurement>& activeContacts,
      const std::vector<bool>& activeFeet);
  void refreshEstimateFromSmoother();
  NavState currentBaseState() const { return optimizedBaseState_; }
  Key currentBaseKey() const { return MakeBaseKey(step_); }
  static Key MakeBaseKey(size_t step) {
    return Symbol('x', static_cast<uint64_t>(step));
  }
  static Key MakeBiasKey() { return Symbol('b', 0); }
  static Key MakeFootKey(size_t foot, size_t episode) {
    return Symbol('f', static_cast<uint64_t>(1000 * foot + episode));
  }
  bool hasPendingImu() const { return pim_.deltaTij() > 0.0; }
  bool graphInitialized() const {
    return !params_.useFullContactInitialization || fullContactInitialized_;
  }
  bool awaitingFullContactInitialization() const {
    return params_.useFullContactInitialization && !fullContactInitialized_;
  }

  size_t numFeet_;
  LeggedEstimatorParams params_;
  std::vector<std::string> footNames_;
  Matrix initialFootholds_;
  Matrix9 baseCovariance0_;
  BatchFixedLagSmoother smoother_;
  PreintegratedImuMeasurements pim_;
  size_t step_ = 0;
  double currentTime_ = 0.0;
  std::vector<bool> inContact_;
  std::vector<bool> initialized_;
  std::vector<size_t> footEpisodes_;
  std::vector<std::optional<Key>> activeFootKeys_;
  NavState optimizedBaseState_;
  NavState deadReckonedState_;
  imuBias::ConstantBias biasEstimate_;
  bool fullContactInitialized_ = false;
};

/**
 * Fixed-lag smoother with CombinedImuFactor bias evolution.
 *
 * This variant keeps the same contact-episode smoother structure as
 * `LeggedFixedLagSmoother`, but replaces the `ImuFactor2` motion model with
 * event-to-event `CombinedImuFactor` links. Pose, velocity, and bias are
 * stored as separate graph variables at each contact event, and the bias is
 * allowed to evolve with a random walk between events.
 *
 * In practice this is the smoother analogue of moving from a fixed,
 * subtractive IMU bias estimate to an explicitly estimated bias trajectory. It
 * serves as an example of how to upgrade a contact-event smoother from simple
 * preintegration with one shared bias key to combined preintegration with
 * per-event bias states.
 */
class GTSAM_EXPORT LeggedCombinedFixedLagSmoother : public LeggedEstimator {
 public:
  /// Construct the combined-IMU fixed-lag smoother variant.
  LeggedCombinedFixedLagSmoother(
      const NavState& navState0, const Matrix& footholds0,
      const Matrix9& baseCovariance0, const LeggedEstimatorParams& params,
      double lagSeconds, const std::vector<std::string>& footNames = {});

  /// Destroy the combined-IMU fixed-lag smoother variant.
  ~LeggedCombinedFixedLagSmoother() override;

  /// Number of feet tracked by the smoother front-end.
  size_t numFeet() const { return numFeet_; }

  /// Current estimate in the shared `ExtendedPose3d` layout.
  ExtendedPose3d estimate() const override;

  /// Return the current per-window bias estimate at the latest event.
  imuBias::ConstantBias estimateBias() const override { return biasEstimate_; }

  /// Accumulate one IMU sample and advance the dead-reckoned estimate.
  void predict(const Vector3& omegaBody, const Vector3& specificForceBody,
               double dt) override;

  /// Process the active contacts for the current step.
  void processContacts(
      const std::vector<ContactMeasurement>& activeContacts) override;

 private:
  bool maybeInitializeFromFullContact(
      const std::vector<ContactMeasurement>& activeContacts,
      const std::vector<bool>& activeFeet);
  void refreshEstimateFromSmoother();
  NavState currentBaseState() const { return optimizedBaseState_; }
  Key currentPoseKey() const { return MakePoseKey(step_); }
  Key currentVelocityKey() const { return MakeVelocityKey(step_); }
  Key currentBiasKey() const { return MakeBiasKey(step_); }
  static Key MakePoseKey(size_t step) {
    return Symbol('x', static_cast<uint64_t>(step));
  }
  static Key MakeVelocityKey(size_t step) {
    return Symbol('v', static_cast<uint64_t>(step));
  }
  static Key MakeBiasKey(size_t step) {
    return Symbol('b', static_cast<uint64_t>(step));
  }
  static Key MakeFootKey(size_t foot, size_t episode) {
    return Symbol('f', static_cast<uint64_t>(1000 * foot + episode));
  }
  bool hasPendingImu() const { return pim_.deltaTij() > 0.0; }
  bool graphInitialized() const {
    return !params_.useFullContactInitialization || fullContactInitialized_;
  }
  bool awaitingFullContactInitialization() const {
    return params_.useFullContactInitialization && !fullContactInitialized_;
  }

  size_t numFeet_;
  LeggedEstimatorParams params_;
  std::vector<std::string> footNames_;
  Matrix initialFootholds_;
  Matrix9 baseCovariance0_;
  BatchFixedLagSmoother smoother_;
  PreintegratedCombinedMeasurements pim_;
  size_t step_ = 0;
  double currentTime_ = 0.0;
  std::vector<bool> inContact_;
  std::vector<bool> initialized_;
  std::vector<size_t> footEpisodes_;
  std::vector<std::optional<Key>> activeFootKeys_;
  NavState optimizedBaseState_;
  NavState deadReckonedState_;
  imuBias::ConstantBias biasEstimate_;
  bool fullContactInitialized_ = false;
};

}  // namespace gtsam
