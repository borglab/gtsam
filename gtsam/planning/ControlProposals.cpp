// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file    ControlProposals.cpp
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#include <gtsam/planning/ControlProposals.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <typeinfo>
#include <unsupported/Eigen/FFT>
#include <utility>
#include <vector>

namespace gtsam {
namespace {

const Matrix& checkedCovariance(const Matrix& covariance) {
  if (!covariance.rows() || covariance.rows() != covariance.cols() ||
      !covariance.allFinite() || !covariance.isApprox(covariance.transpose()) ||
      (covariance.diagonal().array() <= 0).any())
    throw std::invalid_argument(
        "Expected a finite positive-definite control covariance.");
  return covariance;
}

Matrix weightedMean(const Matrix& samples, const Vector& weights,
                    const Matrix& nominal) {
  const Eigen::Index horizon = nominal.cols();
  if (!weights.size() || !nominal.size() || samples.rows() != nominal.rows() ||
      samples.cols() != horizon * weights.size())
    throw std::invalid_argument(
        "Expected complete control sequences and one weight per sequence.");
  Matrix mean = Matrix::Zero(samples.rows(), horizon);
  // An infeasible candidate can contain nonfinite controls. Skip its zero
  // weight before multiplication so it cannot contaminate feasible samples.
  for (Eigen::Index j = 0; j < weights.size(); ++j)
    if (weights[j] != 0)
      mean += weights[j] * samples.middleCols(j * horizon, horizon);
  return mean;
}

Matrix fittedSpread(const Matrix& samples, const Matrix& mean,
                    const Vector& weights, double floor) {
  Matrix variance = Matrix::Zero(mean.rows(), mean.cols());
  for (Eigen::Index j = 0; j < weights.size(); ++j) {
    if (weights[j] > 0)
      variance.array() +=
          weights[j] * (samples.middleCols(j * mean.cols(), mean.cols()) - mean)
                           .array()
                           .square();
  }
  return variance.array().sqrt().max(floor);
}

}  // namespace

GaussianProposal::GaussianProposal(const Matrix& covariance, size_t samples,
                                   std::uint64_t seed,
                                   GaussianProposalOptions options)
    : noise_(checkedCovariance(covariance), samples, seed),
      initialStddev_(covariance.diagonal().array().sqrt()),
      options_(std::move(options)) {
  if (!samples || !(options_.decay > 0 && options_.decay <= 1) ||
      !(options_.zeroMeanFraction >= 0 && options_.zeroMeanFraction <= 1))
    throw std::invalid_argument(
        "Invalid sample count, decay or zero-mean fraction.");
  const auto& coeff = options_.likelihoodCoefficients;
  if (coeff.size() && (coeff.size() != covariance.rows() ||
                       !coeff.allFinite() || !covariance.isDiagonal()))
    throw std::invalid_argument(
        "Upstream likelihood coefficients require a diagonal covariance and "
        "one coefficient per control.");
}

GaussianProposal::State GaussianProposal::initialize(Matrix nominal) const {
  if (!nominal.cols() || nominal.rows() != initialStddev_.size() ||
      !nominal.allFinite())
    throw std::invalid_argument(
        "Expected finite nominal controls matching the covariance.");
  Matrix spread = options_.stddev;
  if (!spread.size()) spread = initialStddev_.replicate(1, nominal.cols());
  if (spread.rows() != nominal.rows() || spread.cols() != nominal.cols() ||
      !spread.allFinite() || (spread.array() <= 0).any())
    throw std::invalid_argument(
        "Expected positive finite standard deviations matching the controls.");
  return {std::move(nominal), std::move(spread)};
}

std::unique_ptr<GaussianProposal> GaussianProposal::clone() const {
  return std::make_unique<GaussianProposal>(*this);
}

Matrix GaussianProposal::drawNoise(Eigen::Index horizon) const {
  return noise_.sampleNoise(horizon).array().colwise() / initialStddev_.array();
}

Matrix GaussianProposal::drawControls(const State& state,
                                      bool includeIncumbent) const {
  const Eigen::Index horizon = state.mean.cols(),
                     count = static_cast<Eigen::Index>(noise_.numSamples());
  if (!horizon || state.mean.rows() != initialStddev_.size() ||
      state.stddev.rows() != state.mean.rows() ||
      state.stddev.cols() != horizon || !state.stddev.allFinite() ||
      (state.stddev.array() < 0.).any())
    throw std::invalid_argument("Proposal state has inconsistent dimensions.");

  // Fast path: fixed white Gaussian preserves seeded draws bit for bit.
  if (typeid(*this) == typeid(GaussianProposal) && options_.decay == 1.0 &&
      options_.zeroMeanFraction == 0.0 &&
      (state.stddev.colwise() - initialStddev_).isZero(0.0)) {
    Matrix controls = noise_.sampleControls(state.mean);
    if (includeIncumbent || options_.includeNominal)
      controls.leftCols(horizon) = state.mean;
    return controls;
  }

  // Draw unit-marginal noise and verify batch shape.
  Matrix controls = drawNoise(horizon);
  if (controls.rows() != state.mean.rows() ||
      controls.cols() != count * horizon)
    throw std::invalid_argument(
        "Noise draws must match the control dimension and batch size.");

  // Apply decayed per-step spread and center the non-zero-mean fraction.
  const Matrix scale = state.stddev * std::pow(options_.decay, state.iteration);
  const double meanCutoff =
      (1.0 - options_.zeroMeanFraction) * static_cast<double>(count);
  for (Eigen::Index j = 0; j < count; ++j) {
    auto sample = controls.middleCols(j * horizon, horizon);
    sample.array() *= scale.array();
    if (static_cast<double>(j) < meanCutoff) sample += state.mean;
  }

  // Reserve candidate 0 for the nominal plan when requested.
  if (includeIncumbent || options_.includeNominal)
    controls.leftCols(horizon) = state.mean;
  return controls;
}

Vector GaussianProposal::logWeightOffsets(const State& state,
                                          const Matrix& controls) const {
  if (!options_.likelihoodCoefficients.size()) return {};
  const Eigen::Index horizon = state.mean.cols(),
                     count = controls.cols() / horizon;
  Vector offsets = Vector::Zero(count);
  const Matrix scaledMean =
      (options_.likelihoodCoefficients.asDiagonal() * state.mean).array() /
      state.stddev.array().square();
  const double meanCutoff =
      (1.0 - options_.zeroMeanFraction) * static_cast<double>(count);
  for (Eigen::Index j = 0; static_cast<double>(j) < meanCutoff; ++j)
    offsets[j] =
        -(scaledMean.array() *
          (controls.middleCols(j * horizon, horizon) - 0.5 * state.mean)
              .array())
             .sum();
  return offsets;
}

ControlBatch GaussianProposal::sampleImpl(const State& state, const Values&,
                                          double, bool incumbent) const {
  if (!state.stddev.size()) {
    auto initialized = initialize(state.mean);
    initialized.iteration = state.iteration;
    return sampleImpl(initialized, {}, 1., incumbent);
  }
  ControlBatch batch;
  batch.controls = drawControls(state, incumbent);
  batch.logWeightOffsets = logWeightOffsets(state, batch.controls);
  return batch;
}

void GaussianProposal::update(State& state, const ControlBatch& batch,
                              const Vector& weights) const {
  state.mean = weightedMean(batch.controls, weights, state.mean);
  ++state.iteration;
}

void GaussianProposal::fitImpl(State& state, const ControlBatch& batch,
                               const Vector& weights, double minStddev) const {
  if (!(minStddev > 0.) || !std::isfinite(minStddev))
    throw std::invalid_argument("Expected a positive finite spread floor.");
  update(state, batch, weights);
  state.stddev = fittedSpread(batch.controls, state.mean, weights, minStddev);
}

ColoredNoiseProposal::ColoredNoiseProposal(const Matrix& covariance,
                                           size_t samples, double exponent,
                                           std::uint64_t seed,
                                           GaussianProposalOptions options)
    : GaussianProposal(covariance, samples, seed, std::move(options)),
      exponent_(exponent) {
  if (!std::isfinite(exponent) || exponent < 0)
    throw std::invalid_argument(
        "Expected a finite nonnegative spectral exponent.");
}

Matrix ColoredNoiseProposal::drawNoise(Eigen::Index horizon) const {
  Matrix noise = GaussianProposal::drawNoise(horizon);
  if (horizon == 1 || exponent_ == 0) return noise;

  // Build 1/f^(exponent/2) amplitude weights normalized to unit marginal power.
  Vector gain(horizon);
  for (Eigen::Index k = 0; k < horizon; ++k) {
    const auto frequency = std::max<Eigen::Index>(1, std::min(k, horizon - k));
    gain[k] = std::pow(frequency, -0.5 * exponent_);
  }
  gain *= std::sqrt(static_cast<double>(horizon) / gain.squaredNorm());

  // Shape each candidate channel independently in the frequency domain.
  Eigen::FFT<double> fft;
  Eigen::VectorXcd spectrum;
  Vector series(horizon);
  for (Eigen::Index j = 0; j < noise.cols(); j += horizon) {
    for (Eigen::Index row = 0; row < noise.rows(); ++row) {
      auto segment = noise.row(row).segment(j, horizon);
      series = segment.transpose();
      fft.fwd(spectrum, series);
      spectrum.array() *= gain.array();
      fft.inv(series, spectrum);
      segment = series.transpose();
    }
  }
  return noise;
}

std::unique_ptr<GaussianProposal> ColoredNoiseProposal::clone() const {
  return std::make_unique<ColoredNoiseProposal>(*this);
}

NormalLognormalProposal::NormalLognormalProposal(
    const Matrix& covariance, size_t samples, double logStddev,
    std::uint64_t seed, GaussianProposalOptions options)
    : GaussianProposal(covariance, samples, seed, std::move(options)),
      logStddev_(logStddev),
      logNoise_(Matrix::Identity(covariance.rows(), covariance.rows()), samples,
                ~seed) {
  if (!covariance.isDiagonal() || !std::isfinite(logStddev) || logStddev < 0)
    throw std::invalid_argument(
        "NLN requires diagonal covariance and finite nonnegative log spread.");
}

Matrix NormalLognormalProposal::drawNoise(Eigen::Index horizon) const {
  Matrix noise = GaussianProposal::drawNoise(horizon);
  noise.array() *= (logStddev_ * logNoise_.sampleNoise(horizon).array() -
                    logStddev_ * logStddev_)
                       .exp();
  return noise;
}

std::unique_ptr<GaussianProposal> NormalLognormalProposal::clone() const {
  return std::make_unique<NormalLognormalProposal>(*this);
}

PiecewiseLinearProposal::PiecewiseLinearProposal(
    const Matrix& covariance, size_t samples, size_t segments,
    std::uint64_t seed, GaussianProposalOptions options)
    : ColoredNoiseProposal(covariance, samples, 0., seed, std::move(options)),
      segments_(segments),
      knotGenerator_(~seed) {
  if (!segments)
    throw std::invalid_argument("Expected at least one linear segment.");
}

PiecewiseLinearProposal::PiecewiseLinearProposal(
    const Matrix& covariance, size_t samples, size_t segments,
    MotionPrimitiveOptions mixture, std::uint64_t seed,
    GaussianProposalOptions options)
    : ColoredNoiseProposal(covariance, samples, mixture.exponent, seed,
                           std::move(options)),
      segments_(segments),
      knotGenerator_(~seed),
      mixture_(std::move(mixture)) {
  auto& p = *mixture_;
  if (!p.amplitude.size()) p.amplitude = covariance.diagonal().array().sqrt();
  if (!segments || samples < 2 || p.amplitude.size() != covariance.rows() ||
      !p.amplitude.allFinite() || (p.amplitude.array() < 0).any() ||
      !(p.coloredFraction >= 0 && p.shiftedFraction >= 0 &&
        p.coloredFraction + p.shiftedFraction <= 1) ||
      !(p.coloredScale >= 0 && std::isfinite(p.coloredScale) &&
        p.shiftedScale >= 0 && std::isfinite(p.shiftedScale)))
    throw std::invalid_argument(
        "Invalid motion primitive mixture or sample count.");
}

std::unique_ptr<GaussianProposal> PiecewiseLinearProposal::clone() const {
  return std::make_unique<PiecewiseLinearProposal>(*this);
}

void PiecewiseLinearProposal::fitImpl(State&, const ControlBatch&,
                                      const Vector&, double) const {
  throw std::invalid_argument(
      "PiecewiseLinearProposal does not support spread-fitting rules.");
}

ControlBatch PiecewiseLinearProposal::sampleImpl(const State& state,
                                                 const Values& initial,
                                                 double dt,
                                                 bool incumbent) const {
  if (!mixture_)
    return GaussianProposal::sampleImpl(state, initial, dt, incumbent);

  const auto& p = *mixture_;
  if (!state.stddev.size()) {
    auto initialized = initialize(state.mean);
    initialized.iteration = state.iteration;
    return sampleImpl(initialized, initial, dt, incumbent);
  }

  const Eigen::Index horizon = state.mean.cols(),
                     count = static_cast<Eigen::Index>(noise_.numSamples());
  if (!horizon || state.mean.rows() != p.amplitude.size() ||
      state.stddev.rows() != state.mean.rows() ||
      state.stddev.cols() != horizon ||
      p.fixedPrefix > static_cast<size_t>(horizon))
    throw std::invalid_argument(
        "Invalid motion primitive horizon or fixed prefix.");

  const auto prefix = static_cast<Eigen::Index>(p.fixedPrefix);
  const double horizonD = static_cast<double>(horizon);
  const double countD = static_cast<double>(count);
  Matrix controls(state.mean.rows(), count * horizon);

  // Pre-generate colored perturbations only when that mixture slice is active.
  Matrix colored;
  if (p.coloredFraction > 0) colored = ColoredNoiseProposal::drawNoise(horizon);
  std::poisson_distribution<size_t> switches(0.5 *
                                             static_cast<double>(segments_));
  std::uniform_real_distribution<double> uniform(0., 1.);

  for (Eigen::Index j = 0; j < count; ++j) {
    auto sample = controls.middleCols(j * horizon, horizon);
    if (j == 0) {
      // Slot 0 always preserves the incumbent trajectory.
      sample = state.mean;
    } else if (j == 1) {
      // Slot 1 reserves an all-zero braking/idle primitive.
      sample.setZero();
    } else if (static_cast<double>(j) < p.coloredFraction * countD) {
      // Colored perturbations around the incumbent.
      sample = state.mean.array() +
               p.coloredScale * state.stddev.array() *
                   colored.middleCols(j * horizon, horizon).array();
    } else {
      // Bounded piecewise-linear primitives (either shifted or absolute).
      const bool shifted = static_cast<double>(j) <
                           (p.coloredFraction + p.shiftedFraction) * countD;
      for (Eigen::Index row = 0; row < sample.rows(); ++row) {
        // Sample sorted interior switch times in (start, 1).
        const size_t knots = std::min(segments_, switches(knotGenerator_));
        const double start = static_cast<double>(prefix) / horizonD;
        std::vector<double> times{start};
        for (size_t i = 0; i < knots; ++i) {
          const double time = uniform(knotGenerator_);
          if (time > start && time < 1.) times.push_back(time);
        }
        std::sort(times.begin(), times.end());
        times.push_back(1.);

        // Interpolate uniform knot values across each time segment.
        auto level = [&] {
          return p.amplitude[row] * (2 * uniform(knotGenerator_) - 1);
        };
        double left = prefix && prefix < horizon
                          ? (shifted ? 0. : state.mean(row, prefix))
                          : level();
        double right = level();
        size_t edge = 0;
        for (Eigen::Index k = prefix; k < horizon; ++k) {
          const double t = static_cast<double>(k) / horizonD;
          while (t >= times[edge + 1]) {
            ++edge;
            left = right;
            right = level();
          }
          const double a = (t - times[edge]) / (times[edge + 1] - times[edge]);
          const double interpolated = (1 - a) * left + a * right;
          sample(row, k) =
              shifted ? state.mean(row, k) + p.shiftedScale * interpolated
                      : interpolated;
        }
      }
    }
    // Lock the requested prefix to the incumbent controls.
    sample.leftCols(prefix) = state.mean.leftCols(prefix);
  }

  Vector offsets = logWeightOffsets(state, controls);
  return {std::move(controls), std::move(offsets)};
}

Matrix PiecewiseLinearProposal::drawNoise(Eigen::Index horizon) const {
  Matrix noise = GaussianProposal::drawNoise(horizon);
  if (static_cast<size_t>(horizon - 1) <= segments_) return noise;

  // Pick random interior knots per candidate and linearly interpolate noise.
  std::vector<Eigen::Index> interior(horizon - 2);
  for (Eigen::Index j = 0; j < noise.cols(); j += horizon) {
    std::iota(interior.begin(), interior.end(), 1);
    std::shuffle(interior.begin(), interior.end(), knotGenerator_);
    std::sort(interior.begin(),
              interior.begin() + static_cast<std::ptrdiff_t>(segments_ - 1));
    Eigen::Index left = 0;
    for (size_t i = 0; i < segments_; ++i) {
      const Eigen::Index right = i + 1 == segments_ ? horizon - 1 : interior[i];
      for (Eigen::Index k = left + 1; k < right; ++k) {
        const double t =
            static_cast<double>(k - left) / static_cast<double>(right - left);
        noise.col(j + k) =
            (1.0 - t) * noise.col(j + left) + t * noise.col(j + right);
      }
      left = right;
    }
  }
  return noise;
}

SmoothControlProposal::State SmoothControlProposal::initialize(
    Matrix nominal) const {
  Matrix derivatives = Matrix::Zero(nominal.rows(), nominal.cols());
  return {GaussianProposal::initialize(std::move(nominal)),
          std::move(derivatives)};
}

SmoothControlProposal::Batch SmoothControlProposal::sample(
    const State& state, const Values&, double dt, bool incumbent) const {
  if (!(dt > 0) || !std::isfinite(dt))
    throw std::invalid_argument("Expected a finite positive dt.");
  if (state.derivativeMean.rows() != state.mean.rows() ||
      state.derivativeMean.cols() != state.mean.cols())
    throw std::invalid_argument(
        "Derivative and control means must have matching dimensions.");
  Batch batch;
  batch.derivatives = drawControls(
      {state.derivativeMean, state.stddev, state.iteration}, false);
  if (incumbent || includesNominal())
    batch.derivatives.leftCols(state.mean.cols()).setZero();
  batch.controls =
      state.mean.replicate(1, static_cast<Eigen::Index>(noise_.numSamples())) +
      dt * batch.derivatives;
  batch.logWeightOffsets = logWeightOffsets(state, batch.controls);
  return batch;
}

void SmoothControlProposal::update(State& state, const Batch& batch,
                                   const Vector& weights) const {
  Matrix derivativeMean =
      weightedMean(batch.derivatives, weights, state.derivativeMean);
  GaussianProposal::update(state, batch, weights);
  state.derivativeMean = std::move(derivativeMean);
}

void SmoothControlProposal::fit(State& state, const Batch& batch,
                                const Vector& weights, double floor) const {
  update(state, batch, weights);
  state.stddev =
      fittedSpread(batch.derivatives, state.derivativeMean, weights, floor);
}

}  // namespace gtsam
