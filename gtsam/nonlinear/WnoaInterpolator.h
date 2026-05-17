/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file WnoaInterpolator.h
 * @brief Interpolator class implementation for interpolating poses and
 * velocities between two bordering states, either during or after a factor
 * graph solve. Interpolator assumes the WNOA motion prior by default.
 * @date June 20, 2025
 * @author Zi Cong Guo
 * @author Connor Holmes
 * @author Sven Lilge
 */
#pragma once

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaStateData.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * @brief Type alias for a pair of optional bordering states for an interpolated
 * state.
 *
 * The pair holds the left and right bordering StateData for an interpolated
 * state. Either entry may be empty (std::nullopt) to indicate a missing
 * border; in that case extrapolation is used instead of interpolation.
 */
using StateDataInterval =
    std::pair<std::optional<StateData>, std::optional<StateData>>;

/**
 * @brief Simple container for a pose and its corresponding velocity.
 *
 * Used as the local representation of a state when interpolating pose and
 * velocity pairs. The velocity uses the tangent vector type associated with
 * the PoseType.
 * @tparam PoseType Pose group/type (e.g., Pose2, Pose3, or a vector-space pose)
 */
template <typename PoseType>
struct PoseVelocity {
  PoseType pose;
  typename traits<PoseType>::TangentVector vel;

  /// Returns pose and velocity as a std::pair.
  std::pair<PoseType, typename traits<PoseType>::TangentVector> asPair() const {
    return std::make_pair(pose, vel);
  }
};

/**
 * @brief Timestamped pose and velocity container.
 *
 * Wraps a `PoseVelocity` together with a timestamp (double seconds). This is
 * the primary representation passed to interpolation routines when computing
 * an interpolated state at a requested time.
 * @tparam PoseType Pose group/type (e.g., Pose2, Pose3, or a vector-space pose)
 */
template <typename PoseType>
struct TimestampedPoseVelocity {
  PoseVelocity<PoseType> poseVel;
  double timestamp;

  TimestampedPoseVelocity(PoseType pose,
                          typename traits<PoseType>::TangentVector vel,
                          double time)
      : poseVel{pose, vel}, timestamp(time) {}

  TimestampedPoseVelocity(PoseVelocity<PoseType> pv, double time)
      : poseVel(pv), timestamp(time) {}
};

/**
 * @brief Type alias for a map from variable keys to interpolated state covariances.
 */
typedef std::map<Key, Matrix> InterpCovarianceMap;

/**
 * @brief Interpolator for poses and velocities under a motion prior.
 *
 * The `Interpolator` class provides routines to interpolate pose and
 * velocity at arbitrary times between (or outside) estimated states. It
 * encapsulates the motion model (WNOA by default) via function objects for
 * transition, covariance and Jacobian computations. The class also computes
 * conditional covariances for interpolated states when requested.
 *
 * @tparam PoseType Pose group/type (e.g., Pose2, Pose3, or a vector-space pose)
 */
template <typename PoseType>
class GTSAM_EXPORT Interpolator {
 protected:
  static constexpr int dim = traits<PoseType>::dimension;
  using VelocityType = typename traits<PoseType>::TangentVector;
  using MatrixN = Eigen::Matrix<double, dim, dim>;
  using VectorN = Eigen::Matrix<double, dim, 1>;
  using Matrix2N = Eigen::Matrix<double, 2 * dim, 2 * dim>;
  using Vector2N = Eigen::Matrix<double, 2 * dim, 1>;
  using MatrixNx2N = Eigen::Matrix<double, dim, 2 * dim>;

  VectorN q_psd_diag_;  // Diagonal power Spectral Density for WNOA
  std::function<Matrix(double dt)> transitionFunction_;
  std::function<Matrix(double dt, const VectorN& q_psd_diag)>
      covarianceFunction_;
  std::function<Matrix(double dt, const VectorN& q_psd_diag)>
      inverseCovarianceFunction_;
  // Todo: need to make the below two functions generalize to cases with no
  // velocities, e.g. WNOV
  std::function<Matrix(const std::pair<PoseType, VelocityType>&,
                       const std::pair<PoseType, VelocityType>&, double)>
      computeJacobianPrev_;
  std::function<Matrix(const std::pair<PoseType, VelocityType>&,
                       const std::pair<PoseType, VelocityType>&, double)>
      computeJacobianNext_;

 public:
  // Type aliases
  using StateDataSet = std::set<StateData>;
  using PoseVel = PoseVelocity<PoseType>;
  using TimestampedPoseVel = TimestampedPoseVelocity<PoseType>;
  using LambdaPsiMats = std::pair<Matrix2N, Matrix2N>;
  using LocalStateVecs = std::pair<VectorN, VectorN>;
  struct StateJacobians {
    MatrixN dxi_dTk;
    MatrixN dxi_dTkp1;
    MatrixN dxidot_dTk;
    MatrixN dxidot_dTkp1;
    MatrixN dxidotkp1_dvarpikp1;
  };


  Interpolator() = delete;

  /**
   * @brief Construct an Interpolator with custom motion-model functions.
   *
   * @param q_psd_diag Diagonal power spectral density vector for the motion
   * prior.
   * @param transitionFunction Function returning the transition matrix for dt.
   * @param covarianceFunction Function returning process noise covariance for
   * dt and q_psd_diag.
   * @param inverseCovarianceFunction Function returning inverse covariance for
   * dt and q_psd_diag.
   * @param computeJacobianPrev Function that computes the Jacobian of the
   * interpolated state with respect to the previous bordering state.
   * @param computeJacobianNext Function that computes the Jacobian of the
   * interpolated state with respect to the next bordering state.
   */
  Interpolator(
      const VectorN& q_psd_diag,
      std::function<Matrix(double dt)> transitionFunction,
      std::function<Matrix(double dt, const VectorN& q_psd_diag)>
          covarianceFunction,
      std::function<Matrix(double dt, const VectorN& q_psd_diag)>
          inverseCovarianceFunction,
      std::function<Matrix(const std::pair<PoseType, VelocityType>&,
                           const std::pair<PoseType, VelocityType>&, double)>
          computeJacobianPrev,
      std::function<Matrix(const std::pair<PoseType, VelocityType>&,
                           const std::pair<PoseType, VelocityType>&, double)>
          computeJacobianNext);

  /**
   * @brief Default constructor using the WNOA motion model.
   *
   * @param q_psd_diag Diagonal power spectral density vector for the motion
   * prior.
   */
  Interpolator(const VectorN& q_psd_diag);

  /**
   * @brief Interpolate the pose and velocity at time `t_tau`.
   *
   * The function computes the interpolated pose and velocity at the requested
   * time using the provided bordering (optional) timestamped states. When a
   * border is missing, boundary or extrapolation logic is used.
   *
   * This is the main workhorse function for interpolated factors, but is also
   * used after the fact to compute interpolated values and covariances given a
   * solved factor graph.
   *
   * Optionally returns Jacobians (8 matrices) and the interpolated covariance
   * when the main-solve marginals are provided.
   *
   * @param Tvarpi_k Optional timestamped pose/velocity at left border t_k.
   * @param Tvarpi_kp1 Optional timestamped pose/velocity at right border t_kp1.
   * @param t_tau Interpolation time.
   * @param H Optional output vector of Jacobian blocks (8 matrices).
   * @param mainSolveMarginalMatrix Optional marginal covariance of bordering
   * states from the main solve.
   * @param covarianceOut Optional output pointer for the interpolated state's
   * covariance (requires mainSolveMarginalMatrix).
   * @param LambdaPsiPreComp Optional precomputed Lambda/Psi matrices to reuse
   * work.
   * @param localStateVecsPreComp Optional precomputed local state vectors (xi,
   * xi_dot).
   * @param stateJacobiansPreComp Optional precomputed local state->global
   * state Jacobians.
   * @return PoseVel Interpolated pose and velocity pair at `t_tau`.
   */
  PoseVel interpolatePoseAndVelocity(
      const std::optional<TimestampedPoseVel>& Tvarpi_k,
      const std::optional<TimestampedPoseVel>& Tvarpi_kp1, double t_tau,
      OptionalMatrixVecType H = nullptr,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix = nullptr,
      Matrix* covarianceOut = nullptr,
      const std::shared_ptr<const LambdaPsiMats>& LambdaPsiPreComp = nullptr,
      const std::shared_ptr<const LocalStateVecs>& localStateVecsPreComp =
          nullptr,
      const std::shared_ptr<const StateJacobians>& stateJacobiansPreComp =
          nullptr) const;

  /**
   * @brief Interpolate multiple poses and velocities given a solved factor
   * graph.
   *
   * This routine takes a factor graph and the solution for the estimated
   * states and returns a `Values` object containing interpolated poses and
   * velocities for the requested set of `interpolatedStates`.
   *
   * Optionally fills `covarianceMapOut` with the conditional covariances for
   * each interpolated state.
   *
   * @param mainSolveGraph Factor graph used in the main solve (estimated states
   * only).
   * @param mainSolveSolution Solution `Values` from the main solve.
   * @param mainSolveStates Ordered set of estimated `StateData` used in the
   * main solve.
   * @param interpolatedStates Set of `StateData` to interpolate.
   * @param covarianceMapOut Optional output map to receive per-state
   * covariances.
   * @return Values A `Values` container with interpolated pose and velocity
   * entries.
   */
  Values interpolatePosesAndVelocities(
      const NonlinearFactorGraph& mainSolveGraph,
      const Values& mainSolveSolution, const StateDataSet& mainSolveStates,
      const StateDataSet& interpolatedStates,
      std::shared_ptr<InterpCovarianceMap> covarianceMapOut = nullptr) const;

  /**
   * @brief Compute the conditional covariance of the interpolated state.
   *
   * Given timestamped left and right bordering states and the interpolated
   * state's timestamped value, compute the 2N x 2N conditional covariance
   * matrix Sigma_tau. Optionally return the interpolation matrices Lambda and
   * Psi.
   *
   * @param pvk Timestamped pose/velocity at left border.
   * @param pvkp1 Timestamped pose/velocity at right border.
   * @param pvtau Timestamped pose/velocity at interpolation time.
   * @param Lambda Optional output pointer to receive Lambda matrix.
   * @param Psi Optional output pointer to receive Psi matrix.
   * @return Matrix2N Conditional covariance matrix of the interpolated state.
   */
  Matrix2N computeConditionalCov(const TimestampedPoseVel& pvk,
                                 const TimestampedPoseVel& pvkp1,
                                 const TimestampedPoseVel& pvtau,
                                 OptionalMatrixType Lambda = nullptr,
                                 OptionalMatrixType Psi = nullptr) const;

  /**
   * @brief Compute interpolation matrices Lambda and Psi (WNOA-optimized).
   *
   * Lambda and Psi relate the bordering states to the interpolated state and
   * are used for both calculating interpolated values and propagating
   * covariance. This implementation is optimized for the WNOA motion prior.
   *
   * @param t_k Time of the left border state.
   * @param t_kp1 Time of the right border state.
   * @param t_tau Interpolation time.
   * @return std::pair<Matrix,Matrix> Pair (Lambda, Psi) interpolation matrices.
   */
  std::pair<Matrix, Matrix> getLambdaPsi(double t_k, double t_kp1,
                                         double t_tau) const;

  /**
   * @brief Compute interpolation matrices Lambda and Psi for a general motion
   * model.
   *
   * This general implementation does not rely on WNOA-specific simplifications
   * and can be used for other motion priors provided the appropriate
   * transition/covariance functions are supplied.
   *
   * @param t_k Time of the left border state.
   * @param t_kp1 Time of the right border state.
   * @param t_tau Interpolation time.
   * @return std::pair<Matrix,Matrix> Pair (Lambda, Psi) interpolation matrices.
   */
  std::pair<Matrix, Matrix> getLambdaPsiGeneral(double t_k, double t_kp1,
                                                double t_tau) const;
  /**
   * @brief Compute the local state vectors (xi, xi_dot) for a pair of bordering
   * states.
   *
   * The local state vectors are used as intermediate quantities in
   * interpolation and in Jacobian computations. Optionally returns
   * local-to-global Jacobians.
   *
   * @param pvk Timestamped pose/velocity at left border state.
   * @param pvkp1 Timestamped pose/velocity at right border state.
   * @param jacs Optional output pointer to receive local-to-global Jacobians.
   * @return LocalStateVecs Pair of local state vectors (xi, xi_dot).
   */
  LocalStateVecs computeLocalStateVecs(const TimestampedPoseVel& pvk,
                                       const TimestampedPoseVel& pvkp1,
                                       StateJacobians* jacs = nullptr) const;

 protected:
  /**
   * @brief Interpolate pose and velocity at the left boundary.
   *
   * When the requested interpolation time lies at the left border
   * state's timestamp, this routine computes the interpolated pose and
   * velocity using boundary logic. Optionally returns Jacobian blocks and
   * the interpolated covariance when the main-solve marginal is provided.
   *
   * @param poseVel_k Pose and velocity at the left border (local
   * representation).
   * @param H Optional output pointer to a vector of Jacobian blocks.
   * @param mainSolveMarginalMatrix Optional shared pointer to the main-solve
   * marginal covariance.
   * @param covarianceOut Optional output pointer for the resulting covariance
   * matrix.
   * @return PoseVel Interpolated pose and velocity at the left boundary.
   */
  PoseVel interpolateBoundaryLeft(
      const PoseVelocity<PoseType>& poseVel_k,
      OptionalMatrixVecType H = nullptr,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix = nullptr,
      Matrix* covarianceOut = nullptr) const;

  /**
   * @brief Interpolate pose and velocity at the right boundary.
   *
   * When the requested interpolation time lies at the right border
   * state's timestamp, this routine computes the interpolated pose and
   * velocity using boundary logic. Optionally returns Jacobian blocks and
   * the interpolated covariance when the main-solve marginal is provided.
   *
   * @param poseVel_kp1 Pose and velocity at the right border (local
   * representation).
   * @param H Optional output pointer to a vector of Jacobian blocks.
   * @param mainSolveMarginalMatrix Optional shared pointer to the main-solve
   * marginal covariance.
   * @param covarianceOut Optional output pointer for the resulting covariance
   * matrix.
   * @return PoseVel Interpolated pose and velocity at the right boundary.
   */
  PoseVel interpolateBoundaryRight(
      const PoseVelocity<PoseType>& poseVel_kp1,
      OptionalMatrixVecType H = nullptr,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix = nullptr,
      Matrix* covarianceOut = nullptr) const;

  /**
   * @brief Extrapolate pose and velocity from a border state.
   *
   * Performs extrapolation when the interpolation time lies outside the
   * interval between bordering states. The input `t_diff` is the time
   * difference between the target time and the provided state's timestamp.
   * Optionally provides Jacobians and covariance output when requested.
   *
   * @param poseVel Pose and velocity at the source border.
   * @param t_diff Time difference from source timestamp to interpolation time.
   * @param H Optional output pointer to a vector of Jacobian blocks.
   * @param mainSolveMarginalMatrix Optional shared pointer to the main-solve
   * marginal covariance.
   * @param covarianceOut Optional output pointer for the resulting covariance
   * matrix.
   * @return PoseVel Extrapolated pose and velocity at the requested time.
   */
  PoseVel extrapolatePoseAndVelocity(
      const PoseVelocity<PoseType>& poseVel, double t_diff,
      OptionalMatrixVecType H = nullptr,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix = nullptr,
      Matrix* covarianceOut = nullptr) const;

  /**
   * @brief Internal implementation that performs interpolation between two
   * bordering states. This function follows the interpolation logic outlined in
   * Section 11.3.1 of (Barfoot 2024) and is used by the public
   * `interpolatePoseAndVelocity` method after handling boundary/extrapolation
   * cases.
   *
   * This internal overload implements the core interpolation algorithm for a
   * pair of timestamped bordering states and a target interpolation time.
   * It supports optional precomputed helpers (Lambda/Psi matrices and local
   * state vectors/Jacobians) to avoid redundant work when called in tight
   * loops.
   *
   * @param tPoseVel_k Timestamped pose/velocity at the left border.
   * @param tPoseVel_kp1 Timestamped pose/velocity at the right border.
   * @param t_tau Interpolation time.
   * @param H Optional output pointer to a vector of Jacobian blocks.
   * @param mainSolveMarginalMatrix Optional shared pointer to the main-solve
   * marginal covariance.
   * @param covarianceOut Optional output pointer for the resulting covariance
   * matrix.
   * @param LambdaPsiPreComp Optional precomputed Lambda/Psi matrices to reuse
   * work.
   * @param localStateVecsPreComp Optional precomputed local state vectors (xi,
   * xi_dot).
   * @param stateJacobiansPreComp Optional precomputed local->global
   * Jacobians.
   * @return PoseVel Interpolated pose and velocity at `t_tau`.
   */
  PoseVel interpolatePoseAndVelocity_(
      const TimestampedPoseVel& tPoseVel_k,
      const TimestampedPoseVel& tPoseVel_kp1, double t_tau,
      OptionalMatrixVecType H = nullptr,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix = nullptr,
      Matrix* covarianceOut = nullptr,
      const std::shared_ptr<const LambdaPsiMats>& LambdaPsiPreComp = nullptr,
      const std::shared_ptr<const LocalStateVecs>& localStateVecsPreComp =
          nullptr,
      const std::shared_ptr<const StateJacobians>& stateJacobiansPreComp =
          nullptr) const;

  /**
   * @brief Step 1 of interpolatePoseAndVelocity: form local state vectors and
   * optional local Jacobians.
   *
   * Computes $(\xi_k, \dot\xi_k, \xi_{k+1}, \dot\xi_{k+1})$ used by
   * interpolation. Jacobian outputs are populated only when all Jacobian
   * pointers are non-null.
   *
   * @param tPoseVel_k Timestamped left border state.
   * @param tPoseVel_kp1 Timestamped right border state.
   * @param localStateVecsPreComp Optional precomputed local state vectors at
   * the right border.
   * @param stateJacobiansPreComp Optional precomputed local-to-global
   * Jacobians.
   * @param xi_dot_k Output local velocity vector at the left border.
   * @param xi_kp1 Output local position vector at the right border.
   * @param xi_dot_kp1 Output local velocity vector at the right border.
   * @param jacs Optional output Jacobians for local state wrt bordering states.
   * @return void
   */
  void formLocalStateAndJacobians_(
      const TimestampedPoseVel& tPoseVel_k,
      const TimestampedPoseVel& tPoseVel_kp1,
      const std::shared_ptr<const LocalStateVecs>& localStateVecsPreComp,
      const std::shared_ptr<const StateJacobians>& stateJacobiansPreComp,
      VectorN* xi_dot_k, VectorN* xi_kp1, VectorN* xi_dot_kp1,
      StateJacobians* jacs = nullptr) const;

  /**
   * @brief Step 2 of interpolatePoseAndVelocity: interpolate local state using
   * Lambda/Psi matrices.
   *
   * Retrieves (or reuses precomputed) Lambda/Psi and computes
   * $(\xi_\tau, \dot\xi_\tau)$.
   *
   * @param t_k Left border timestamp.
   * @param t_kp1 Right border timestamp.
   * @param t_tau Query timestamp.
   * @param xi_dot_k Local velocity vector at the left border.
   * @param xi_kp1 Local position vector at the right border.
   * @param xi_dot_kp1 Local velocity vector at the right border.
   * @param LambdaPsiPreComp Optional precomputed $(\Lambda, \Psi)$ pair.
   * @param Lambda Output interpolation matrix $\Lambda$.
   * @param Psi Output interpolation matrix $\Psi$.
   * @param xi_tau Output interpolated local position vector.
   * @param xidot_tau Output interpolated local velocity vector.
   * @return void
   */
  void interpolateLocalState_(
      double t_k, double t_kp1, double t_tau, const VectorN& xi_dot_k,
      const VectorN& xi_kp1, const VectorN& xi_dot_kp1,
      const std::shared_ptr<const LambdaPsiMats>& LambdaPsiPreComp,
      Matrix2N* Lambda, Matrix2N* Psi, VectorN* xi_tau,
      VectorN* xidot_tau) const;

  /**
   * @brief Step 3 of interpolatePoseAndVelocity: map interpolated local state
   * back to manifold.
   *
   * Computes interpolated pose/velocity pair. Jacobian outputs are optional
   * and populated only when `dTtau_dTk` and `dTtau_dxitau` are non-null.
   *
   * @param T_k Left border pose.
   * @param xi_tau Interpolated local position vector.
   * @param xidot_tau Interpolated local velocity vector.
   * @param right_jac_tau Output right Jacobian of `Expmap(xi_tau)`.
   * @param dTtau_dTk Optional output Jacobian of $T_\tau$ wrt $T_k$.
   * @param dTtau_dxitau Optional output Jacobian of $T_\tau$ wrt $\xi_\tau$.
   * @return PoseVel Interpolated pose and velocity pair at $t_\tau$.
   */
  PoseVel mapLocalStateToManifold_(const PoseType& T_k, const VectorN& xi_tau,
                                   const VectorN& xidot_tau,
                                   MatrixN* right_jac_tau,
                                   MatrixN* dTtau_dTk = nullptr,
                                   MatrixN* dTtau_dxitau = nullptr) const;

  /**
   * @brief Step 4 of interpolatePoseAndVelocity: compose full Jacobians with
   * chain rule.
   *
   * @param Lambda Interpolation matrix $\Lambda$.
   * @param Psi Interpolation matrix $\Psi$.
   * @param xidot_tau Interpolated local velocity vector.
   * @param right_jac_tau Right Jacobian of `Expmap(xi_tau)`.
   * @param dTtau_dTk Jacobian of $T_\tau$ wrt $T_k$ from composition.
   * @param dTtau_dxitau Jacobian of $T_\tau$ wrt $\xi_\tau$.
   * @param jacs Jacobians for local state wrt bordering states.
   * @param H Output vector of Jacobian blocks to populate.
   * @return void
   */
  void computeCompleteJacobians_(const Matrix2N& Lambda, const Matrix2N& Psi,
                                 const VectorN& xidot_tau,
                                 const MatrixN& right_jac_tau,
                                 const MatrixN& dTtau_dTk,
                                 const MatrixN& dTtau_dxitau,
                                 const StateJacobians& jacs,
                                 OptionalMatrixVecType H) const;

  /**
   * @brief Step 5 of interpolatePoseAndVelocity: compute interpolated
   * covariance when requested.
   *
   * @param tPoseVel_k Timestamped left border state.
   * @param tPoseVel_kp1 Timestamped right border state.
   * @param poseVel_tau Interpolated pose/velocity at query time.
   * @param t_tau Query timestamp.
   * @param Lambda Interpolation matrix $\Lambda$.
   * @param Psi Interpolation matrix $\Psi$.
   * @param mainSolveMarginalMatrix Optional covariance of bordering states.
   * @param covarianceOut Optional output covariance for interpolated state.
   * @return void
   */
  void computeInterpolationCovariance_(
      const TimestampedPoseVel& tPoseVel_k,
      const TimestampedPoseVel& tPoseVel_kp1, const PoseVel& poseVel_tau,
      double t_tau, const Matrix2N& Lambda, const Matrix2N& Psi,
      const std::shared_ptr<Matrix>& mainSolveMarginalMatrix,
      Matrix* covarianceOut) const;

  /**
   * @brief Compute joint marginal covariances for requested state intervals.
   *
   * Given a set of query buckets mapping intervals to the contained
   * `StateData` entries, extract and assemble the joint marginal covariance
   * matrices from the provided `Marginals` object. Returns a map from the
   * interval to the joint marginal matrix (as a shared pointer).
   *
   * @param queryBuckets Mapping of `StateDataInterval` to a vector of
   * `StateData` keys.
   * @param marginals Unique pointer to a `Marginals` object computed from the
   * main solve.
   * @return std::map<StateDataInterval, std::shared_ptr<Matrix>> Map of joint
   * marginal matrices.
   */
  static std::map<StateDataInterval, std::shared_ptr<Matrix>>
  ComputeJointMarginals(
      const std::map<StateDataInterval, std::vector<StateData>>& queryBuckets,
      const std::unique_ptr<Marginals>& marginals);

  /**
   * @brief Construct a full covariance matrix from a joint marginal block
   * matrix.
   *
   * The `blockMatrix` argument contains the joint marginal arranged in a
   * block structure. This helper reorders and flattens it into a full dense
   * matrix according to `keyVector` and the specified `blockSize`.
   *
   * @param blockMatrix The joint marginal in block structure.
   * @param keyVector Ordering of keys to determine row/column ordering.
   * @param blockSize Size of each block (e.g., dimension of a single state
   * block).
   * @return Matrix Full covariance matrix assembled from the joint marginal.
   */
  static Matrix ConstructMatrixFromJointMarginal(
      const JointMarginal& blockMatrix, const KeyVector& keyVector,
      size_t blockSize);
};

}  // namespace gtsam
