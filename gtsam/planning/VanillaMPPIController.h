/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VanillaMPPIController.h
 * @brief   Vanilla MPPI controller: the generic MPPI controller with Gaussian
 *          control noise, and its parameters
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/planning/GaussianNoiseSampler.h>
#include <gtsam/planning/GenericMPPIController.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace gtsam {

/**
 * Parameters of the vanilla MPPI controller: the generic MPPI parameters
 * plus those of its GaussianNoiseSampler.
 */
struct VanillaMPPIParams : public GenericMPPIParams {
  size_t numSamples = 1024;  ///< Number of sampled control sequences.
  Matrix controlCovariance;  ///< Control noise covariance; must be set.
  std::uint64_t seed = 42;   ///< Seed of the sampling generators.
};

/**
 * MPPI controller with Gaussian control noise. The sampler uses the
 * controlCovariance, numSamples and seed from the controller parameters.
 *
 * Each iteration of optimize() is the information-theoretic MPPI update of
 * [1, 2]. K control sequences v_i = ubar + eps_i are sampled, with eps_i
 * drawn from N(0, controlCovariance) at every step, and each is rolled out
 * and scored with S_i, the error of the future graph. The nominal controls
 * become the weighted mean ubar = sum_i w_i v_i, with
 * w_i = exp(-(S_i - min S) / lambda) normalized and
 * lambda = 1 / inverseTemperature. The weights have no importance-sampling
 * control-cost term: this is the case alpha = 1, so
 * gamma = lambda (1 - alpha) = 0, of [2], where the base distribution is that
 * of the current plan. Control costs are factors of the future graph on the
 * sampled controls.
 *
 * REFERENCES:
 * [1] G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots
 *     and E. A. Theodorou, "Information Theoretic MPC for Model-Based
 *     Reinforcement Learning", ICRA, pp. 1714-1721, 2017.
 * [2] G. Williams, P. Drews, B. Goldfain, J. M. Rehg and E. A. Theodorou,
 *     "Information-Theoretic Model Predictive Control: Theory and
 *     Applications to Autonomous Driving", IEEE Transactions on Robotics,
 *     34(6):1603-1622, 2018.
 *
 * @tparam DYNAMICS_FACTOR The rollout factor described by GenericMPPIController.
 */
template <class DYNAMICS_FACTOR>
class VanillaMPPIController
    : public GenericMPPIController<DYNAMICS_FACTOR, GaussianNoiseSampler> {
 public:
  /// The generic controller with the Gaussian noise sampler.
  using Base = GenericMPPIController<DYNAMICS_FACTOR, GaussianNoiseSampler>;

  /**
   * Construct with the given parameters. controlCovariance may be left
   * empty, but sampling then throws.
   * @throws std::invalid_argument if controlCovariance is set but is not a
   * square positive-definite matrix of size ControlDim x ControlDim.
   */
  explicit VanillaMPPIController(
      const VanillaMPPIParams& params = VanillaMPPIParams())
      : Base(params, GaussianNoiseSampler(params.controlCovariance,
                                          params.numSamples, params.seed)),
        params_(params) {
    if (this->sampler().dim() != 0 &&
        this->sampler().dim() != static_cast<size_t>(Base::ControlDim)) {
      throw std::invalid_argument(
          "VanillaMPPIController: controlCovariance must match the dimension "
          "of the control.");
    }
  }

  /// The controller's parameters.
  const VanillaMPPIParams& params() const { return params_; }

  /// The number of sampling calls so far; see GaussianNoiseSampler.
  std::uint64_t stepCounter() const { return this->sampler().stepCounter(); }

  /// Sample the control noise over the horizon; see GaussianNoiseSampler.
  const Matrix& sampleNoise(size_t horizon) const {
    return this->sampler().sampleNoise(horizon);
  }

 protected:
  /// The parameters, as the base parameter type.
  const MPCControllerParams& _params() const override { return params_; }

 private:
  const VanillaMPPIParams params_;  ///< Vanilla MPPI parameters.
};

}  // namespace gtsam
