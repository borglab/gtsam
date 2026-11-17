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
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam_unstable/mpc_stag/controller/GenericMPPIController.h>
#include <gtsam_unstable/mpc_stag/controller/sampler/GaussianNoiseSampler.h>

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
 * Vanilla MPPI controller on the future half of the STAG graph: the
 * GenericMPPIController with Gaussian control noise, from a
 * GaussianNoiseSampler built from the parameters' controlCovariance,
 * numSamples and seed.
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
 * @tparam STATE    State type of the future graph's dynamics factors.
 * @tparam CONTROL  Control type, a fixed-size Eigen vector such as Vector2.
 */
template <class STATE, class CONTROL>
class VanillaMPPIController
    : public GenericMPPIController<STATE, CONTROL, GaussianNoiseSampler> {
 public:
  /// The generic controller with the Gaussian noise sampler.
  using Base = GenericMPPIController<STATE, CONTROL, GaussianNoiseSampler>;

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
          "of CONTROL.");
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
  const StagMPCControllerParams& _params() const override { return params_; }

 private:
  const VanillaMPPIParams params_;  ///< Vanilla MPPI parameters.
};

}  // namespace gtsam
