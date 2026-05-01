/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    WnoaInterpFactor.h
 * @brief   White-Noise-On-Acceleration (WNOA) continuous time interpolation
 * wrapper factor and functions to automatically update a given graph by
 * interpolating some variables (removing them from the optimization).
 * @author  Connor Holmes
 * @author  Sven Lilge
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Point1.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>
#include <gtsam/nonlinear/WnoaStateData.h>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * @class WNOAInterpFactor
 * @brief Wrapper factor that evaluates an inner NoiseModelFactor on states
 *        interpolated from neighboring estimated states using a WNOA
 *        (white-noise-on-acceleration) Gaussian process interpolator.
 *
 * Template parameter:
 *   - PoseType: Pose type for states (e.g., Pose2, Pose3) used by the
 * interpolator.
 *
 * Description:
 *   This factor presents an interface identical to a regular factor defined
 *   on "interpolated" pose/velocity keys but internally maps those keys to
 *   surrounding estimated (outer) states. It:
 *     - builds interpolated Values for requested query times,
 *     - evaluates the inner factor on interpolated states,
 *     - backpropagates inner-factor Jacobians through the interpolation
 *       mapping to form Jacobians w.r.t. outer estimated keys,
 *     - optionally augments the measurement noise by adding the
 *       interpolation-induced covariance (conditional covariances),
 *       resulting in an effective Gaussian noise model used for whitening.
 *
 * Key features:
 *   - Supports precomputation/reuse via PassedInterpData (interpolated Values,
 *     flattened jacobian blocks, and conditional covariances).
 *   - Option to keep the original noise model fixed (no covariance
 * augmentation).
 *   - Optional precomputation of interpolation Lambda/Psi matrices
 *   - Exposes helpers to obtain mappings between interpolated keys and their
 *     bordering estimated states.
 *
 * Typical usage:
 *   Construct by providing an inner NoiseModelFactor, ordered sets of
 *   estimated and interpolated StateData, and the WNOA PSD vector q_psd_diag.
 *   Use as a drop-in factor in a graph that references only estimated keys;
 *   call linearize/error/noiseModel as usual. When many factors share
 *   interpolation, supply PassedInterpData to avoid redundant work.
 *
 * Preconditions / Notes:
 *   - Estimated and interpolated StateData must be time-ordered and
 *     interpolation query times must lie strictly between bordering estimated
 * times.
 *   - PoseType must be either a Lie-group or vector-space type (checked at
 * compile time).
 */
template <class PoseType>
class WNOAInterpFactor : public NoiseModelFactor {
 private:
#ifndef GTSAM_ROT3_EXPMAP
  static_assert(
      !std::is_same_v<PoseType, Pose3> || std::is_same_v<PoseType, Rot3>,
      "Interpolation factors are not supported for Pose3/Rot3 when using the "
      "Cayley chart. Please switch to EXPMAP coordinates mode.");
#endif
  using Base = NoiseModelFactor;
  using This = WNOAInterpFactor<PoseType>;
  using VelocityType = typename gtsam::traits<PoseType>::TangentVector;
  static constexpr int dim = traits<PoseType>::dimension;

  // Convenient matrices
  using Matrix2N = Eigen::Matrix<double, 2 * dim, 2 * dim>;
  using MatrixN = Eigen::Matrix<double, dim, dim>;
  using LambdaPsiMats = typename Interpolator<PoseType>::LambdaPsiMats;

  // Tell the compiler to import the base class's version of error
  // Note: this is required because we don't define error( const HybridValues& )
  // in this class, and without this line, the base class's error(const
  // HybridValues&) is hidden by the error(const Values&, OptionalMatrixVecType)
  // defined in this class.
  using Base::error;

  // Inner factor that is called on interpolated values
  const NoiseModelFactor::shared_ptr inner_factor_;
  // Interpolator object for the given PoseType
  const Interpolator<PoseType> interpolator_;
  // disable noise model updates
  const bool fixed_noise_model_;
  // map keys to interpolated state
  std::unordered_map<Key, StateData> key_to_interp_;
  // map interpolated state to border states.
  std::unordered_map<StateData, std::pair<StateData, StateData>>
      interp_to_borders_;
  // map outer key to outer key index (for Jacobians)
  std::unordered_map<Key, int> outer_key_to_index_;
  // map of precomputed matrices for interpolation, keyed by StateData
  std::unordered_map<StateData, std::shared_ptr<LambdaPsiMats>>
      lambda_psi_pre_comp_;

  // Cache inner key -> index mapping to avoid rebuilding in noise model calc
  std::unordered_map<Key, int> inner_key_to_index_;

  // This struct contains the indices of the contributing outer keys as well as
  // flags for whether the inner key is interpolated or not. Keeps track of
  // which estimated states contribute to a given interpolated key, and the
  // corresponding Jacobian blocks for efficient linearization.
  struct InnerKeyMapping {
    bool isInterpolated = false;
    // For non-interpolated keys
    int directOuterIndex = -1;
    // For interpolated keys: cached outer indices and keys
    int indexPoseLeft = -1, indexVelLeft = -1, indexPoseRight = -1,
        indexVelRight = -1;
    Key keyPoseLeft = 0, keyVelLeft = 0, keyPoseRight = 0, keyVelRight = 0;
  };
  std::vector<InnerKeyMapping> inner_key_mappings_;

 public:
  /**
   * @brief Container for externally-computed interpolation data.
   *
   * When available, callers can precompute interpolated `Values`, flattened
   * inner->outer jacobian blocks and conditional covariances and pass them
   * into `linearize`/`error` to avoid recomputation inside the factor.
   *
   * Used for efficient value reuse when many factors share the same
   * interpolation (e.g., multiple measurement factors referencing the same
   * interpolated state).
   */
  struct PassedInterpData {
    /// Interpolated Values (pose and velocity entries at interpolated times)
    Values values;

    /**
     * Flattened jacobian blocks for each inner key mapping to its contributing
     * outer keys. The array order is [LPose, LVel, RPose, RVel].
     */
    std::unordered_map<Key, std::array<Matrix, 4>> jacobians;

    /// Conditional covariance (2N x 2N) for each interpolated StateData.
    std::unordered_map<StateData, Matrix2N> condCovs;
  };

  /**
   * @brief Construct a WNOA interpolation wrapper factor.
   *
   * The wrapper maps an inner factor that references "interpolated" states
   * onto the outer estimated states used by the main solve.
   *
   * @param inner_factor Factor defined over (possibly) interpolated keys.
   * @param estimated_states Ordered set of estimated `StateData`. These are the
   * states that are directly optimized over and serve as borders for
   * interpolation.
   * @param interp_states Ordered set of states to interpolate (states at query
   * times).
   * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior.
   * @param fixed_noise_model If true, do not augment the noise measurement
   * model with GP interpolation process noise. This is faster, but less
   * accurate, as it ignores interpolation uncertainty.
   * @param precomp_interp_mats If true, precompute Lambda/Psi matrices for each
   * interpolated state.
   */
  WNOAInterpFactor(const NoiseModelFactor::shared_ptr inner_factor,
                   const std::set<StateData> estimated_states,
                   const std::set<StateData> interp_states,
                   const Eigen::Matrix<double, dim, 1> q_psd_diag,
                   const bool fixed_noise_model = false,
                   const bool precomp_interp_mats = true)
      : Base(inner_factor->noiseModel()),
        inner_factor_(inner_factor),
        interpolator_(q_psd_diag),
        fixed_noise_model_(fixed_noise_model) {
    // PROCESS INTERPOLATED STATES
    // loop through interpolated states
    for (const StateData& state : interp_states) {
      // search for estimated state that upper bound current interpolated state
      // Note: lower_bound finds the least upper bound using time-based
      // comparator of StateData. We can use it to find the right border state
      // for interpolation.
      auto iter_est_state = estimated_states.lower_bound(state);
      // Check if right border state is out of bounds (i.e., interp time is
      // outside the range of estimated times)
      if (iter_est_state == estimated_states.begin()) {
        throw std::runtime_error(
            "Interpolated state time is before all estimated state times");
      } else if (iter_est_state == estimated_states.end()) {
        throw std::runtime_error(
            "Interpolated state time is after all estimated state times");
      } else {
        // decrement iterator (points to left border state)
        iter_est_state--;
        // map interpolated state to borderstates
        interp_to_borders_[state] =
            std::pair(*iter_est_state, *std::next(iter_est_state));
        // Keep track of the inner keys corresponding to a given interpolated
        // state for easy lookup when building outer keys and mapping jacobians
        // later
        key_to_interp_[state.pose] = state;
        key_to_interp_[state.velocity] = state;
      }
      // Precompute Lambda and Psi WNOA interpolation matrices
      if (precomp_interp_mats) {
        lambda_psi_pre_comp_[state] =
            std::make_shared<LambdaPsiMats>(interpolator_.getLambdaPsi(
                interp_to_borders_[state].first.time,
                interp_to_borders_[state].second.time, state.time));
      } else {
        lambda_psi_pre_comp_[state] = nullptr;
      }
    }
    // DEFINE KEYS
    // Define set of "outer" keys that this wrapper factor is defined on.
    std::unordered_set<Key> outer_key_set;
    for (Key key : inner_factor->keys()) {
      if (key_to_interp_.find(key) == key_to_interp_.end()) {
        // inner key is not interpolated, add to outer keys
        outer_key_set.insert(key);
      } else {
        // inner key is interpolated, add associated border state keys to this
        // factor's keys
        StateData& interp = key_to_interp_.at(key);          // get state
        auto [left, right] = interp_to_borders_.at(interp);  // get borders
        outer_key_set.insert(left.pose);                     // add border keys
        outer_key_set.insert(left.velocity);
        outer_key_set.insert(right.pose);
        outer_key_set.insert(right.velocity);
      }
    }
    // Convert to key vector (from set)
    keys_ = KeyVector(outer_key_set.begin(), outer_key_set.end());

    // map outer keys to their associated index (used when mapping jacobians
    // later)
    for (size_t i = 0; i < this->keys_.size(); i++) {
      outer_key_to_index_[this->keys_[i]] = i;
    }
    // Build inner key mappings once and cache inner key indices
    // We will use this mapping to know how to map inner Jacobian blocks to
    // outer

    // number of inner keys (keys associated with just the inner factor)
    const KeyVector& inner_keys_init = inner_factor_->keys();

    // Vector of structs that stores the mapping information for each inner key
    // (whether it's interpolated, and the corresponding outer key indices)
    inner_key_mappings_.resize(inner_keys_init.size());

    // This map allows us to quickly find the index of an inner key in the inner
    // factor's key ordering This is important for correctly mapping Jacobian
    // blocks during linearization. We build this map once in the constructor to
    // avoid redundant work later (slight speed up)
    inner_key_to_index_.reserve(inner_keys_init.size());

    // Loop through all inner keys and save the relevant mappings to outer keys
    for (size_t i = 0; i < inner_keys_init.size(); ++i) {
      // inner key
      Key innerKey = inner_keys_init[i];
      // current index
      inner_key_to_index_[innerKey] = static_cast<int>(i);
      InnerKeyMapping mapping;
      auto itInterp = key_to_interp_.find(innerKey);

      // If this condition is met, then this inner key is not interpolated and
      // directly corresponds to an outer key We can map the Jacobian block for
      // this inner key directly to the corresponding outer key index
      if (itInterp == key_to_interp_.end()) {
        auto itOuter = outer_key_to_index_.find(innerKey);
        if (itOuter != outer_key_to_index_.end())
          mapping.directOuterIndex = itOuter->second;
      } else {
        // Otherwise, this inner key is interpolated and we need to map it to
        // its corresponding border states The Jacobian blocks that connect them
        // to the inner key
        mapping.isInterpolated = true;

        // get border states for this interpolated key using the
        // interp_to_borders_ map we built earlier
        const StateData& sd = itInterp->second;
        const auto& br = interp_to_borders_.at(sd);
        const StateData& left = br.first;
        const StateData& right = br.second;

        // Map the border states to their corresponding outer key indices
        // This uses the outer_key_to_index_ map we built earlier
        // We will need these indices to know where to map Jacobian blocks
        // during linearization
        mapping.indexPoseLeft = outer_key_to_index_.at(left.pose);
        mapping.indexVelLeft = outer_key_to_index_.at(left.velocity);
        mapping.indexPoseRight = outer_key_to_index_.at(right.pose);
        mapping.indexVelRight = outer_key_to_index_.at(right.velocity);

        // also save the actual keys for clarity (might not be accessed during
        // compute)
        mapping.keyPoseLeft = left.pose;
        mapping.keyVelLeft = left.velocity;
        mapping.keyPoseRight = right.pose;
        mapping.keyVelRight = right.velocity;
      }
      inner_key_mappings_[i] = mapping;
    }
  };

  /**
   * @brief Default destructor.
   */
  ~WNOAInterpFactor() override {};

  /** Implement functions needed for Testable */
  /**
   * @brief Print factor information for debugging.
   *
   * Prints the outer keys this wrapper connects and forwards a print of the
   * inner factor for detailed inspection.
   */
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "WNOAInterpFactor on ";
    for (const auto& k : this->keys()) {
      std::cout << keyFormatter(k) << " ";  // raw numeric key
    }
    std::cout << std::endl;
    this->inner_factor_->print("Inner Factor: ");
  }
  /**
   * @brief Equality test used by unit tests and debug checks.
   */
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol);
  }

  /**
   * @brief Expose base class overloads for `unwhitenedError`.
   */
  using Base::unwhitenedError;

  /**
   * @brief Compute the unwhitened residual vector for the wrapped inner factor.
   *
   * This is the outer-facing residual (before whitening) obtained by
   * evaluating the inner factor on interpolated states and mapping the
   * resulting error back to the outer state ordering.
   */
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    return computeInterpolatedError(values, H);
  }

  /**
   * @brief Linearize the wrapper factor, computing a JacobianFactor.
   *
   * When interpolation increases measurement uncertainty, this function
   * updates the noise model on-the-fly by computing the inner-factor
   * Jacobians with respect to the outer estimated states and the
   * interpolated conditional covariances.
   */
  std::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    // Only linearize if the factor is active
    if (!active(x)) return std::shared_ptr<JacobianFactor>();

    // Compute residual and effective noise model.
    std::vector<Matrix> A(size());
    Vector b;
    auto noise_model = eval(x, nullptr, b, &A);
    return makeJacobianFactor(A, b, noise_model);
  }

  /**
   * @brief Linearize using externally provided interpolation data.
   *
   * This overload accepts a `PassedInterpData` pointer which can contain
   * precomputed interpolated `Values`, flattened jacobians and conditional
   * covariances. Using externally-computed data avoids duplicate work when
   * many factors share interpolation computations.
   */
  std::shared_ptr<GaussianFactor> linearize(
      const Values& x, PassedInterpData* passedInterpData) const {
    // Only linearize if the factor is active
    if (!active(x)) return std::shared_ptr<JacobianFactor>();

    // Compute residual and effective noise model.
    std::vector<Matrix> A(size());
    Vector b;
    auto noise_model = eval(x, passedInterpData, b, &A);
    return makeJacobianFactor(A, b, noise_model);
  }

  /**
   * @brief Compute the factor error (unwhitened) using the current `Values`.
   *
   * This variant computes the scalar loss by forming the residual vector and
   * evaluating the (possibly augmented) noise model.
   */
  double error(const Values& c) const override {
    if (!active(c)) return 0.0;

    Vector b;
    auto noise_model = eval(c, nullptr, b, nullptr);
    return loss(b, noise_model);
  }

  /**
   * @brief Error computation accepting precomputed interpolation data.
   *
   * Behaves like `error(const Values&)` but reads jacobians and conditional
   * covariances from `passedInterpData` to avoid recomputation.
   */
  double error(const Values& c, PassedInterpData* passedInterpData) const {
    if (!active(c)) return 0.0;

    Vector b;
    auto noise_model = eval(c, passedInterpData, b, nullptr);
    return loss(b, noise_model);
  }

  /**
   * @brief Return an augmented noise model accounting for interpolation.
   *
   * Computes inner-factor jacobians and conditional covariances, then uses
   * them to form the effective Gaussian noise model that should be used for
   * whitening the residuals. If `fixed_noise_model_` is set this returns the
   * base factor's Gaussian noise model unchanged.
   */
  SharedGaussian noiseModel(Values& x) const {
    // if fixed noise then just return the standard gaussian model
    if (fixed_noise_model_) {
      return std::dynamic_pointer_cast<noiseModel::Gaussian>(
          Base::noiseModel());
    }
    // Call evaluate error to get inner Jacobians and convariances
    std::vector<Matrix> JacInner(inner_factor_->size());
    std::unordered_map<StateData, Matrix2N> InterpCondCovs;
    Vector b =
        -computeInterpolatedError(x, nullptr, &JacInner, &InterpCondCovs);
    // get interpolated noise model
    return getInterpolatedNoiseModel(JacInner, InterpCondCovs);
  }

  /**
   * @brief Get the mapping from outer Keys to their corresponding interpolated
   * StateData.
   * @return unordered_map<Key, StateData>
   */
  std::unordered_map<Key, StateData> getInterpolatedKeys() const {
    return key_to_interp_;
  }

  /**
   * @brief Get the mapping from each interpolated StateData to its bordering
   * states.
   * @return unordered_map<StateData, pair<StateData, StateData>>
   */
  std::unordered_map<StateData, std::pair<StateData, StateData>>
  getInterpToBorders() const {
    return interp_to_borders_;
  }

 private:
  /// Compute unwhitened residual and matching Gaussian noise model.
  noiseModel::Gaussian::shared_ptr eval(const Values& values,
                                        PassedInterpData* passedInterpData,
                                        Vector& b,
                                        std::vector<Matrix>* A) const {
    if (A && (A->size() != size())) A->resize(size());

    return fixed_noise_model_ ? evalFixed(values, A, passedInterpData, b)
                              : evalInterp(values, A, passedInterpData, b);
  }

  /// Compute residual/noise model for fixed-noise mode.
  noiseModel::Gaussian::shared_ptr evalFixed(const Values& values,
                                             std::vector<Matrix>* A,
                                             PassedInterpData* passedInterpData,
                                             Vector& b) const {
    b = -computeInterpolatedError(values, A, nullptr, nullptr,
                                  passedInterpData);
    return std::dynamic_pointer_cast<noiseModel::Gaussian>(Base::noiseModel());
  }

  /// Compute residual/noise model for interpolation-augmented noise.
  noiseModel::Gaussian::shared_ptr evalInterp(
      const Values& values, std::vector<Matrix>* A,
      PassedInterpData* passedInterpData, Vector& b) const {
    std::vector<Matrix> jacInner(inner_factor_->size());

    if (passedInterpData) {
      b = -computeInterpolatedError(values, A, &jacInner, nullptr,
                                    passedInterpData);
      return getInterpolatedNoiseModel(jacInner, passedInterpData->condCovs);
    }

    std::unordered_map<StateData, Matrix2N> localInterpCondCovs;
    b = -computeInterpolatedError(values, A, &jacInner, &localInterpCondCovs);
    return getInterpolatedNoiseModel(jacInner, localInterpCondCovs);
  }

  /// Build a JacobianFactor from outer Jacobians and residual.
  std::shared_ptr<GaussianFactor> makeJacobianFactor(
      std::vector<Matrix>& A, Vector& b,
      const noiseModel::Gaussian::shared_ptr& noise_model) const {
    noise_model->WhitenSystem(A, b);

    std::vector<std::pair<Key, Matrix>> terms(size());
    for (size_t j = 0; j < size(); ++j) {
      terms[j].first = keys()[j];
      terms[j].second.swap(A[j]);
    }

    using noiseModel::Constrained;
    if (noiseModel_ && noiseModel_->isConstrained()) {
      return std::make_shared<JacobianFactor>(
          terms, b, std::static_pointer_cast<Constrained>(noiseModel_)->unit());
    }
    return std::make_shared<JacobianFactor>(terms, b);
  }

  /// Compute scalar error from residual and Gaussian noise model.
  double loss(const Vector& b,
              const noiseModel::Gaussian::shared_ptr& noise_model) const {
    if (noise_model)
      return noise_model->loss(noise_model->squaredMahalanobisDistance(b));
    return 0.5 * b.squaredNorm();
  }

  /**
   * @brief Core routine that evaluates the inner factor on interpolated states
   * and maps results to the outer state ordering.
   *
   * This function performs the following steps:
   * - Build or read interpolated `Values` for all requested query times.
   * - Evaluate the inner factor on those interpolated states to obtain the
   *   residual and inner Jacobians.
   * - Propagate the inner Jacobians through interpolation to produce the
   *   outer Jacobian blocks and (optionally) the interpolated conditional
   *   covariances.
   *
   * @param values Outer `Values` containing estimated states.
   * @param H Optional output pointer to outer Jacobian blocks (one per outer
   * key).
   * @param H_inner Optional output pointer to inner factor Jacobian blocks.
   * @param InterpCondCovs Optional output pointer to receive
   * per-interpolated-state conditional covariances.
   * @param passedInterpData Optional input/output pointer with
   * externally-provided interpolated data.
   * @return Vector Unwhitened residual vector mapped to outer ordering.
   */
  Vector computeInterpolatedError(
      const Values& values, OptionalMatrixVecType H = nullptr,
      OptionalMatrixVecType H_inner = nullptr,
      std::unordered_map<StateData, Matrix2N>* InterpCondCovs = nullptr,
      PassedInterpData* passedInterpData = nullptr) const {
    // Interpolation Jacobians stored as flattened map: per interpolated key ->
    // 4 blocks
    std::unordered_map<Key, std::array<Matrix, 4>> interpJacobiansLocal;
    Values valuesInterpLocal;

    std::unordered_map<Key, std::array<Matrix, 4>>* InterpJacobians = nullptr;
    Values* values_interp = nullptr;

    if (passedInterpData) {
      values_interp = &passedInterpData->values;
      if (H) InterpJacobians = &passedInterpData->jacobians;
      if (InterpCondCovs) {
        InterpCondCovs = &passedInterpData->condCovs;
      }
    } else {
      if (H) {
        InterpJacobians = &interpJacobiansLocal;
        valuesInterpLocal =
            getInterpolatedValues(values, InterpJacobians, InterpCondCovs);
        values_interp = &valuesInterpLocal;
      } else {
        valuesInterpLocal =
            getInterpolatedValues(values, nullptr, InterpCondCovs);
        values_interp = &valuesInterpLocal;
      }
    }

    // cache inner keys once
    const KeyVector& inner_keys = inner_factor_->keys();

    // construct values for inner factor using mappings
    Values values_inner;
    for (size_t i = 0; i < inner_keys.size(); ++i) {
      Key key = inner_keys[i];
      if (inner_key_mappings_[i].isInterpolated) {
        auto it_interp = values_interp->find(key);
        if (it_interp == values_interp->end())
          throw std::runtime_error("Interpolated key missing in values_interp");
        values_inner.insert(key, it_interp->value);
      } else {
        auto it_outer = values.find(key);
        if (it_outer == values.end())
          throw std::runtime_error("Key " + DefaultKeyFormatter(key) +
                                   " not found in outer values");
        values_inner.insert(key, it_outer->value);
      }
    }

    // Call inner factor error function with interpolated values.
    std::vector<Matrix> H_inner_local;
    Vector error;
    if (!H_inner) {
      // if H_inner not passed in, use local variable.
      H_inner_local.resize(inner_keys.size());
      H_inner = &H_inner_local;
    }
    if (H || !fixed_noise_model_) {
      error = inner_factor_->unwhitenedError(values_inner, H_inner);
    } else {
      error = inner_factor_->unwhitenedError(values_inner);
    }

    // compute Jacobians for outer keys
    if (H) {
      // loop through inner keys and update outer keys via backpropagation
      // NOTE: it is possible for two inner keys to affect the same outer key
      for (size_t i = 0; i < inner_keys.size(); i++) {
        const Key inner_key = inner_keys[i];
        const Matrix& Jinner = (*H_inner)[i];
        const InnerKeyMapping& mapping = inner_key_mappings_[i];
        if (mapping.isInterpolated) {
          const std::array<Matrix, 4>& J4 = InterpJacobians->at(inner_key);
          // Order: 0:LPose, 1:LVel, 2:RPose, 3:RVel
          if (mapping.indexPoseLeft >= 0) {
            const Matrix& Jblock = J4[0];
            if ((*H)[mapping.indexPoseLeft].size() == 0)
              (*H)[mapping.indexPoseLeft].setZero(Jinner.rows(), Jblock.cols());
            (*H)[mapping.indexPoseLeft].noalias() += Jinner * Jblock;
          }
          if (mapping.indexVelLeft >= 0) {
            const Matrix& Jblock = J4[1];
            if ((*H)[mapping.indexVelLeft].size() == 0)
              (*H)[mapping.indexVelLeft].setZero(Jinner.rows(), Jblock.cols());
            (*H)[mapping.indexVelLeft].noalias() += Jinner * Jblock;
          }
          if (mapping.indexPoseRight >= 0) {
            const Matrix& Jblock = J4[2];
            if ((*H)[mapping.indexPoseRight].size() == 0)
              (*H)[mapping.indexPoseRight].setZero(Jinner.rows(),
                                                   Jblock.cols());
            (*H)[mapping.indexPoseRight].noalias() += Jinner * Jblock;
          }
          if (mapping.indexVelRight >= 0) {
            const Matrix& Jblock = J4[3];
            if ((*H)[mapping.indexVelRight].size() == 0)
              (*H)[mapping.indexVelRight].setZero(Jinner.rows(), Jblock.cols());
            (*H)[mapping.indexVelRight].noalias() += Jinner * Jblock;
          }
        } else {
          const int k = mapping.directOuterIndex;
          if ((*H)[k].size() == 0)
            (*H)[k].setZero(Jinner.rows(), Jinner.cols());
          (*H)[k].noalias() += Jinner;
        }
      }
    }

    return error;
  }

  /**
   * @brief Compute interpolated state values from bordering estimated states.
   *
   * For every `StateData` in `interp_to_borders_` this routine evaluates the
   * interpolator using the left/right bordering estimated states from
   * `values`, inserts the resulting pose and velocity into the returned
   * `Values` container, and optionally returns per-interpolated-state
   * Jacobians and conditional covariances.
   *
   * @param values Outer `Values` containing estimated (outer) states used as
   * borders.
   * @param InterpJacobians Optional output pointer which, if non-null, will be
   *        populated with flattened jacobian blocks for each interpolated key.
   *        Each entry maps a Key to an `std::array<Matrix,4>` ordered as
   *        [LPose, LVel, RPose, RVel]. These blocks map perturbations in the
   *        bordering states to the interpolated inner state.
   * @param InterpCondCovs Optional output pointer which, if non-null, will be
   *        populated with the 2N x 2N conditional covariance matrix for each
   *        interpolated `StateData` (used when forming the augmented noise
   * model).
   * @return Values A `Values` container holding the interpolated pose and
   *         velocity entries (keys are the interpolated state keys).
   */
  Values getInterpolatedValues(
      const Values& values,
      std::unordered_map<Key, std::array<Matrix, 4>>* InterpJacobians = nullptr,
      std::unordered_map<StateData, Matrix2N>* InterpCondCovs = nullptr) const {
    Values values_interp;  // interpolated values

    // loop through interpolated state map and compute values
    for (const auto& [interp_state, border_states] : interp_to_borders_) {
      // unpack border states
      auto& [left, right] = border_states;
      // retrieve estimated state values
      const auto state_left = TimestampedPoseVelocity<PoseType>(
          values.at<PoseType>(left.pose),
          values.at<VelocityType>(left.velocity), left.time);

      const auto state_right = TimestampedPoseVelocity<PoseType>(
          values.at<PoseType>(right.pose),
          values.at<VelocityType>(right.velocity), right.time);

      // Get interpolated state velocity pair
      PoseVelocity<PoseType> result;

      std::vector<Matrix> H(8);
      if (InterpJacobians) {
        result = interpolator_.interpolatePoseAndVelocity(
            state_left, state_right, interp_state.time, &H, nullptr, nullptr,
            lambda_psi_pre_comp_.at(interp_state));
      } else {
        result = interpolator_.interpolatePoseAndVelocity(
            state_left, state_right, interp_state.time, nullptr, nullptr,
            nullptr, lambda_psi_pre_comp_.at(interp_state));
      }

      // insert into values structure
      values_interp.insert(interp_state.pose, result.pose);
      values_interp.insert(interp_state.velocity, result.vel);

      // arrange jacobians in flattened map (fixed order blocks)
      if (InterpJacobians) {
        (*InterpJacobians)[interp_state.pose] =
            std::array<Matrix, 4>{H[0], H[1], H[2], H[3]};
        (*InterpJacobians)[interp_state.velocity] =
            std::array<Matrix, 4>{H[4], H[5], H[6], H[7]};
      }

      // Conditional covariance of interpolated states for noise model update
      if (InterpCondCovs) {
        auto state_tau =
            TimestampedPoseVelocity<PoseType>(result, interp_state.time);
        Matrix2N Sigma_tau = interpolator_.computeConditionalCov(
            state_left, state_right, state_tau);
        (*InterpCondCovs)[interp_state] =
            Sigma_tau;  // assumed preallocated vector
      }
    }

    return values_interp;
  }

  /**
   * @brief Build an augmented Gaussian noise model that accounts for GP
   * interpolation.
   *
   * Starting from the inner factor's measurement covariance, this helper
   * computes the additional covariance induced by interpolating states and
   * propagating inner-factor Jacobians through the interpolation mapping.
   * The result is a Gaussian noise model with covariance
   * Cov_meas + sum(G_tau * Sigma_tau * G_tau^T), where G_tau are the inner
   * factor Jacobian blocks w.r.t. pose/vel and Sigma_tau are the conditional
   * covariances of the interpolated states.
   *
   * @param Jacobians Vector of inner-factor Jacobian blocks (one per inner
   * key), in the same order as `inner_factor_->keys()`.
   * @param InterpCondCovs Map from `StateData` (interpolated) to its 2N x 2N
   *        conditional covariance matrix Sigma_tau.
   * @return SharedGaussian A new Gaussian noise model with the augmented
   * covariance.
   */
  SharedGaussian getInterpolatedNoiseModel(
      const std::vector<Matrix>& Jacobians,
      const std::unordered_map<StateData, Matrix2N>& InterpCondCovs) const {
    // Get noise model of inner factor
    noiseModel::Gaussian::shared_ptr noise_model_ptr =
        std::dynamic_pointer_cast<noiseModel::Gaussian>(
            inner_factor_->noiseModel());
    // Check that the measurement noise is set up as a gaussian
    assert(noise_model_ptr &&
           "Noise model of inner factor must be noiseModel::Gaussian or "
           "derivative");

    // Initialize new covariance with the existing measurement covariance
    int err_dim = noise_model_ptr->dim();
    Matrix noise_cov = noise_model_ptr->covariance();

    // Use cached mapping from inner keys to indices for Jacobian lookup
    // Compute the covariance update based on interpolated states
    // Note: Here, we leverage the block-diagonal approximation of the
    // interpolated covariances (i.e., independence approximation)
    for (auto& [state, borders] : interp_to_borders_) {
      // Retrieve Jacobians from inner factor
      Matrix G_pose(err_dim, dim);
      Matrix G_vel(err_dim, dim);
      auto itPose = inner_key_to_index_.find(state.pose);
      if (itPose != inner_key_to_index_.end()) {
        G_pose = Jacobians[itPose->second];
      } else {
        G_pose.setZero();
      }
      auto itVel = inner_key_to_index_.find(state.velocity);
      if (itVel != inner_key_to_index_.end()) {
        G_vel = Jacobians[itVel->second];
      } else {
        G_vel.setZero();
      }
      Matrix G_tau(err_dim, 2 * dim);
      G_tau << G_pose, G_vel;
      // add covariance

      const Matrix2N& Sigma_tau = InterpCondCovs.at(state);
      noise_cov += G_tau * Sigma_tau * G_tau.transpose();
    }

    // Return interpolated noise model
    return noiseModel::Gaussian::Covariance(noise_cov);
  }
};

/**
 * @brief Convert a factor graph by removing interpolated states.
 *
 * This helper replaces factors that reference interpolated states with
 * equivalent wrapper factors that act on the bordering estimated states.
 * Additionally, WNOA motion prior factors are added between successive
 * estimated states. Factors that do not reference interpolated states are
 * copied unchanged into the returned graph.
 *
 * @tparam PoseType Pose type used in the graph (e.g. `Pose2`, `Pose3`).
 * @param graph Input factor graph possibly containing factors on interpolated
 * states.
 * @param estimated_states Ordered set of estimated `StateData` (main-solve
 * states).
 * @param interp_states Ordered set of `StateData` entries to be
 * interpolated/removed.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior (dimension
 * must match PoseType).
 * @param fixed_noise If true, do not augment measurement noise models for
 * interpolation.
 * @return NonlinearFactorGraph New graph with interpolated states removed and
 * wrapper factors added.
 */
template <class PoseType>
NonlinearFactorGraph interpolateFactorGraph(
    const NonlinearFactorGraph& graph,
    const std::set<StateData>& estimated_states,
    const std::set<StateData>& interp_states, Vector q_psd_diag,
    bool fixed_noise = false) {
  // assert that the pose is the right kind of variable
  static_assert(
      std::is_same_v<typename traits<PoseType>::structure_category,
                     lie_group_tag> ||
          std::is_same_v<typename traits<PoseType>::structure_category,
                         vector_space_tag>,
      "Pose type must be either a Lie group or vector space");
  // check dimension on the power spectral density matrix
  assert(traits<PoseType>::dimension == q_psd_diag.size());
  // Create new factor graph
  NonlinearFactorGraph new_graph;
  // Add WNOA prior between all estimated states
  auto iter_state = estimated_states.begin();
  while (std::next(iter_state) != estimated_states.end()) {
    StateData state_k = *iter_state;
    StateData state_kp1 = *std::next(iter_state);
    // get time diff
    double del_t = state_kp1.time - state_k.time;
    // add factor
    auto motion_factor = std::make_shared<WNOAMotionFactor<PoseType>>(
        state_k.pose, state_k.velocity, state_kp1.pose, state_kp1.velocity,
        del_t, q_psd_diag);
    new_graph.add(motion_factor);
    iter_state++;
  }
  // Get map from keys to interpolated state, and interpolated state to
  // estimated state.
  std::unordered_map<Key, StateData> key_to_interp_;
  std::unordered_map<StateData, std::pair<StateData, StateData>>
      interp_to_borders_;
  for (const StateData& state : interp_states) {
    // search for estimated state that upper bound current interpolated state
    auto iter_est_state = estimated_states.lower_bound(state);
    if (iter_est_state == estimated_states.begin()) {
      throw std::runtime_error(
          "Interpolated state time is before all estimated state times");
    } else if (iter_est_state == estimated_states.end()) {
      throw std::runtime_error(
          "Interpolated state time is after all estimated state times");
    } else {
      // decrement iterator (point to left border)
      iter_est_state--;
      // map interp to left border index
      interp_to_borders_[state] =
          std::pair(*iter_est_state, *std::next(iter_est_state));
      // map keys to interp state
      key_to_interp_[state.pose] = state;
      key_to_interp_[state.velocity] = state;
    }
  }
  // loop through factors and wrap factors on interpolated states
  for (auto& factor : graph) {
    // handle null factor
    if (!factor) continue;
    // if the factor is a WNOA motion factor, do not add it
    if (std::dynamic_pointer_cast<WNOAMotionFactor<PoseType>>(factor)) continue;
    // get ordered sets of interpolated and estimated states
    std::set<StateData> factor_interp_states;
    std::set<StateData> factor_estimated_states;
    for (Key& key : factor->keys()) {
      // check if key is an interpolated value
      if (key_to_interp_.count(key) > 0) {
        // add indices
        StateData interp_state = key_to_interp_[key];
        factor_interp_states.insert(interp_state);
        auto [left, right] = interp_to_borders_.at(interp_state);
        factor_estimated_states.insert(left);
        factor_estimated_states.insert(right);
      }
    }
    // add factor to new graph
    if (factor_interp_states.size() == 0) {
      // factor does not require interpolation, just add factor as is
      new_graph.add(factor);
    } else {
      // Downcast the NonlinearFactor to a NoiseModelFactor
      auto nmfactor = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      assert(nmfactor &&
             "Defined factors must be NoiseModelFactor or derivative class");

      // Define and add factor to new graph
      const auto wrapped_factor = std::make_shared<WNOAInterpFactor<PoseType>>(
          nmfactor, factor_estimated_states, factor_interp_states, q_psd_diag,
          fixed_noise);
      new_graph.add(wrapped_factor);
    }
  }

  return new_graph;
}

/**
 * @brief Update a `Values` with interpolated pose and velocity entries.
 *
 * Given an interpolated factor graph and the current estimated `Values`,
 * evaluate the interpolator to produce interpolated pose/velocity entries
 * and merge them into a copy of `values` which is returned.
 *
 * @tparam PoseType Pose type used by the interpolator.
 * @param interp_graph Factor graph containing interpolation metadata (borders).
 * @param values Current estimated `Values` (outer states used as borders).
 * @param estim_states Ordered set of estimated `StateData` used by the main
 * solve.
 * @param interp_states Ordered set of `StateData` entries to interpolate.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior.
 * @param covarianceMapOut Optional output pointer to receive
 * per-interpolated-state covariances.
 * @return Values A copy of `values` updated with interpolated pose and velocity
 * entries.
 */
template <class PoseType>
Values updateInterpValues(
    const NonlinearFactorGraph& interp_graph, const Values& values,
    const std::set<StateData>& estim_states,
    const std::set<StateData>& interp_states, const Vector Q_psd,
    std::shared_ptr<typename Interpolator<PoseType>::CovarianceMap>
        covarianceMapOut = nullptr) {
  // assert that the pose is the right kind of variable
  static_assert(
      std::is_same_v<typename traits<PoseType>::structure_category,
                     lie_group_tag> ||
          std::is_same_v<typename traits<PoseType>::structure_category,
                         vector_space_tag>,
      "Pose type must be either a Lie group or vector space");
  // check dimension on the power spectral density matrix
  assert(traits<PoseType>::dimension == Q_psd.size());
  // Define interpolator
  Interpolator<PoseType> interpolator(Q_psd);
  // get interpolated values
  Values interp_vals = interpolator.interpolatePosesAndVelocities(
      interp_graph, values, estim_states, interp_states, covarianceMapOut);
  // update values
  Values values_updated(values);
  values_updated.insert(interp_vals);
  return values_updated;
}

/// traits
template <class POSE>
struct traits<WNOAInterpFactor<POSE>>
    : public Testable<WNOAInterpFactor<POSE>> {};

}  // namespace gtsam
