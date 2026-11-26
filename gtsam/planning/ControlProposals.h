// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file    ControlProposals.h
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/planning/GaussianNoiseSampler.h>

#include <functional>
#include <memory>
#include <optional>

namespace gtsam {

/// One column per physical control, grouped by candidate and then time.
struct ControlBatch {
  Matrix controls;
  Vector logWeightOffsets{};  ///< Optional additive log weights for MPPI.
};

/// Gaussian proposal state for one solve.
struct ControlDistribution {
  Matrix mean;      ///< Nominal physical controls.
  Matrix stddev{};  ///< Marginal spread per coordinate and time.
  size_t iteration = 0;
};

/// Sampling options shared by the Gaussian family of trajectory proposals.
struct GaussianProposalOptions {
  Matrix stddev;       ///< Optional M x H initial marginal standard deviations.
  double decay = 1.0;  ///< Multiply sampling amplitude by decay^iteration.
  double zeroMeanFraction = 0.0;  ///< Fraction drawn about zero.
  bool includeNominal = false;    ///< Reserve the first candidate for the mean.
  /// Optional MPPI-Generic control_cost_coeff scaled by (1-alpha). For its
  /// horizon-averaged objective, divide these coefficients and graph cost by H.
  Vector likelihoodCoefficients;
};

/// Additive Gaussian trajectories with a fixed cross-control correlation.
class GTSAM_EXPORT GaussianProposal {
 public:
  using State = ControlDistribution;

  /// Continue drawing from an existing sampler at its current RNG position.
  explicit GaussianProposal(GaussianNoiseSampler sampler)
      : noise_(std::move(sampler)),
        initialStddev_(noise_.standardDeviations()) {}

  const GaussianNoiseSampler& sampler() const { return noise_; }

  /// Set the initial single-step covariance and the number of candidates.
  GaussianProposal(const Matrix& covariance, size_t samples,
                   std::uint64_t seed = 42,
                   GaussianProposalOptions options = {});
  virtual ~GaussianProposal() = default;

  /** Copy the proposal at its current RNG position for a coupled rollout.
   * Derived distributions used through a Gaussian reference must override
   * this operation to preserve their sampling law.
   */
  virtual std::unique_ptr<GaussianProposal> clone() const;

  /// Initialize the sampling distribution from an M x H control sequence.
  State initialize(Matrix nominal) const;

  /// Draw a batch. Predictive sampling reserves the first slot for the
  /// incumbent.
  ControlBatch sample(const State& state, const Values& initial, double dt,
                      bool includeIncumbent = false) const {
    return sampleImpl(state, initial, dt, includeIncumbent);
  }

  /// Replace the nominal controls with a weighted mean and advance one
  /// iteration.
  void update(State& state, const ControlBatch& batch,
              const Vector& weights) const;

  /// Fit weighted marginal means and spreads, retaining the correlation model.
  void fit(State& state, const ControlBatch& batch, const Vector& weights,
           double minStddev) const {
    fitImpl(state, batch, weights, minStddev);
  }

 protected:
  /// Polymorphic sampling implementation used by GaussianProposal::sample.
  virtual ControlBatch sampleImpl(const State& state, const Values& initial,
                                  double dt, bool includeIncumbent) const;
  /// Polymorphic fit implementation used by GaussianProposal::fit.
  virtual void fitImpl(State& state, const ControlBatch& batch,
                       const Vector& weights, double minStddev) const;

  /// Generate unit-marginal perturbations for the complete horizon.
  virtual Matrix drawNoise(Eigen::Index horizon) const;

  /// Add scaled perturbations to centers and optionally reserve the incumbent.
  Matrix drawControls(const State& state, bool includeIncumbent) const;
  /// Compute the optional upstream likelihood convention on physical controls.
  Vector logWeightOffsets(const State& state, const Matrix& controls) const;
  /// Whether to include the physical nominal sequence as the first sample.
  bool includesNominal() const { return options_.includeNominal; }
  GaussianNoiseSampler noise_;

 private:
  Vector initialStddev_;
  GaussianProposalOptions options_;
};

/// Gaussian perturbations with temporal power spectrum proportional to
/// 1/f^exponent.
class GTSAM_EXPORT ColoredNoiseProposal : public GaussianProposal {
 public:
  /// A positive exponent emphasizes low frequencies. Zero gives white noise.
  ColoredNoiseProposal(const Matrix& covariance, size_t samples,
                       double exponent, std::uint64_t seed = 42,
                       GaussianProposalOptions options = {});
  std::unique_ptr<GaussianProposal> clone() const override;

 protected:
  Matrix drawNoise(Eigen::Index horizon) const override;

 private:
  double exponent_;
};

/// Normal perturbations multiplied by independent lognormal scales.
class GTSAM_EXPORT NormalLognormalProposal : public GaussianProposal {
 public:
  /// Set log-scale dispersion. Normalize the product to unit marginal variance.
  NormalLognormalProposal(const Matrix& covariance, size_t samples,
                          double logStddev, std::uint64_t seed = 42,
                          GaussianProposalOptions options = {});
  std::unique_ptr<GaussianProposal> clone() const override;

 protected:
  Matrix drawNoise(Eigen::Index horizon) const override;

 private:
  double logStddev_;
  GaussianNoiseSampler logNoise_;
};

/** MPPI-Generic primitive mixture. Each channel has independent, uniformly
 * timed switches and bounded uniform knot values. A fraction of candidates
 * add colored noise to the incumbent, a second fraction add primitives to
 * it, and the remainder use absolute primitives.
 */
struct MotionPrimitiveOptions {
  Vector amplitude;  ///< Bounds on absolute knot values, per input. Defaults to
                     ///< the control standard deviations.
  double coloredFraction =
      0.;  ///< Fraction sampled with colored perturbations.
  double shiftedFraction = 0.;  ///< Fraction sampled with additive primitives.
  double coloredScale = 1.;     ///< Multiplier on colored standard deviations.
  double shiftedScale = 1.;  ///< Multiplier on additive primitive amplitudes.
  double exponent = 0.;      ///< Temporal power spectrum of colored draws.
  size_t fixedPrefix =
      0;  ///< Number of incumbent inputs to retain in each draw.
};

/// Piecewise-linear Gaussian perturbations, or a mixture of bounded motion
/// primitives.
class GTSAM_EXPORT PiecewiseLinearProposal : public ColoredNoiseProposal {
 public:
  /// Draw up to this many segments with random interior knots on the time grid.
  PiecewiseLinearProposal(const Matrix& covariance, size_t samples,
                          size_t segments, std::uint64_t seed = 42,
                          GaussianProposalOptions options = {});
  /// Draw the MPPI-Generic mixture with up to segments Poisson-distributed
  /// switches.
  PiecewiseLinearProposal(const Matrix& covariance, size_t samples,
                          size_t segments, MotionPrimitiveOptions mixture,
                          std::uint64_t seed = 42,
                          GaussianProposalOptions options = {});
  std::unique_ptr<GaussianProposal> clone() const override;

 protected:
  /// Draw complete controls, reserving the incumbent and zero in mixture mode.
  ControlBatch sampleImpl(const State& state, const Values& initial, double dt,
                          bool includeIncumbent) const override;
  /// Fitting a distribution over knot locations and values is not supported.
  void fitImpl(State&, const ControlBatch&, const Vector&,
               double) const override;
  Matrix drawNoise(Eigen::Index horizon) const override;

 private:
  size_t segments_;
  mutable std::mt19937_64 knotGenerator_;
  std::optional<MotionPrimitiveOptions> mixture_;
};

/// Smooth-MPPI proposal state, carrying physical controls and derivative mean.
struct SmoothControlProposalState : ControlDistribution {
  Matrix derivativeMean;
};

/// Smooth-MPPI samples in action-derivative coordinates and scores physical
/// controls.
class GTSAM_EXPORT SmoothControlProposal : protected GaussianProposal {
 public:
  using GaussianProposal::GaussianProposal;
  using State = SmoothControlProposalState;
  struct Batch : ControlBatch {
    Matrix derivatives;
  };

  /// Start from the supplied controls and zero derivative mean.
  State initialize(Matrix nominal) const;
  /// Realize each derivative draw as nominal + dt * derivative.
  Batch sample(const State& state, const Values& initial, double dt,
               bool includeIncumbent) const;
  /// Retain the weighted derivative mean alongside the new control sequence.
  void update(State& state, const Batch& batch, const Vector& weights) const;
  /// Fit the spread in derivative coordinates.
  void fit(State& state, const Batch& batch, const Vector& weights,
           double minStddev) const;
};

}  // namespace gtsam
