// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file GenerativePredictiveControl.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#include <gtsam_unstable/planning/GenerativePredictiveControl.h>
#include <gtsam_unstable/planning/internal/AdamUpdate.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam {
namespace {

// Bound products before conversion to signed Eigen dimensions.
Eigen::Index checkedProduct(size_t a, size_t b) {
  if (!a || !b ||
      a > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max()) / b)
    throw std::invalid_argument(
        "GPC dimensions must have a positive representable product.");
  return static_cast<Eigen::Index>(a * b);
}

void checkFlow(double warmStart, size_t steps) {
  if (!(warmStart >= 0 && warmStart <= 1) || !steps)
    throw std::invalid_argument(
        "GPC needs warmStart in [0,1] and at least one flow step.");
}

size_t sampleCount(size_t gaussian, size_t policy) {
  if (gaussian > std::numeric_limits<size_t>::max() - policy ||
      gaussian + policy == 0)
    throw std::invalid_argument("Invalid GPC candidate counts.");
  return gaussian + policy;
}

Vector normalDraw(size_t dimension, std::mt19937_64& generator) {
  std::normal_distribution<double> normal;
  Vector draw(dimension);
  for (Eigen::Index i = 0; i < draw.size(); ++i) draw[i] = normal(generator);
  return draw;
}

}  // namespace

GenerativePredictiveControl::GenerativePredictiveControl(size_t observationDim,
                                                         size_t controlDim,
                                                         size_t horizon,
                                                         size_t hiddenDim,
                                                         size_t seed)
    : observationDim_(checkedProduct(observationDim, 1)),
      controlDim_(checkedProduct(controlDim, 1)),
      horizon_(checkedProduct(horizon, 1)),
      generator_(seed) {
  const auto outputDim = checkedProduct(controlDim, horizon);
  const size_t maxIndex =
      static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
  if (!observationDim || observationDim >= maxIndex - outputDim)
    throw std::invalid_argument("Invalid GPC observation dimension.");
  const auto inputDim = outputDim + observationDim + 1;
  checkedProduct(hiddenDim, inputDim);
  inputWeights_ = Matrix(hiddenDim, inputDim);
  outputWeights_ = Matrix::Zero(outputDim, inputWeights_.rows());
  hiddenBias_ = Vector::Zero(inputWeights_.rows());
  outputBias_ = Vector::Zero(outputDim);
  std::normal_distribution<double> init(0., 1. / std::sqrt(inputDim));
  for (Eigen::Index i = 0; i < inputWeights_.size(); ++i)
    inputWeights_.data()[i] = init(generator_);
}

Vector GenerativePredictiveControl::features(const Vector& sequence,
                                             const Vector& observation,
                                             double time) const {
  Vector result(inputWeights_.cols());
  result << sequence, observation, time;
  return result;
}

void GenerativePredictiveControl::checkInputs(const Vector& observation,
                                              const Matrix& previous) const {
  if (observation.size() != observationDim_ || previous.rows() != controlDim_ ||
      previous.cols() != horizon_ || !observation.allFinite() ||
      !previous.allFinite())
    throw std::invalid_argument(
        "GPC observation or previous plan has invalid shape or values.");
}

Matrix GenerativePredictiveControl::generate(const Vector& observation,
                                             const Matrix& previous,
                                             double warmStart,
                                             size_t flowSteps) const {
  return generateWithRng(observation, previous, warmStart, flowSteps,
                         generator_);
}

Matrix GenerativePredictiveControl::generateWithRng(
    const Vector& observation, const Matrix& previous, double warmStart,
    size_t flowSteps, std::mt19937_64& generator) const {
  checkInputs(observation, previous);
  checkFlow(warmStart, flowSteps);
  Vector sequence =
      warmStart * Eigen::Map<const Vector>(previous.data(), previous.size());
  if (warmStart < 1.)
    sequence += (1. - warmStart) * normalDraw(previous.size(), generator);
  const double step = 1. / static_cast<double>(flowSteps);
  for (size_t i = 0; i < flowSteps; ++i) {
    const Vector input =
        features(sequence, observation, static_cast<double>(i) * step);
    const Vector hidden = (inputWeights_ * input + hiddenBias_).array().tanh();
    sequence += step * (outputWeights_ * hidden + outputBias_);
  }
  return Eigen::Map<const Matrix>(sequence.data(), controlDim_, horizon_);
}

Matrix GenerativePredictiveControl::candidates(
    const Vector& observation, const Matrix& previous, size_t gaussianSamples,
    size_t policySamples, double explorationStddev, double warmStart,
    size_t flowSteps) const {
  checkInputs(observation, previous);
  checkFlow(warmStart, flowSteps);
  if (!(explorationStddev >= 0.) || !std::isfinite(explorationStddev))
    throw std::invalid_argument(
        "GPC exploration spread must be finite and nonnegative.");
  const size_t count = sampleCount(gaussianSamples, policySamples);
  Matrix result(controlDim_, checkedProduct(horizon_, count));
  for (size_t i = 0; i < gaussianSamples; ++i) {
    const Vector draw = normalDraw(previous.size(), generator_);
    controlTrajectory(result, static_cast<Eigen::Index>(i), horizon_) =
        previous + explorationStddev * Eigen::Map<const Matrix>(
                                           draw.data(), controlDim_, horizon_);
  }
  for (size_t i = gaussianSamples; i < count; ++i)
    controlTrajectory(result, static_cast<Eigen::Index>(i), horizon_) =
        generate(observation, previous, warmStart, flowSteps);
  return result;
}

double GenerativePredictiveControl::fit(const Matrix& observations,
                                        const Matrix& previousPlans,
                                        const Matrix& targets,
                                        size_t gradientSteps, size_t batchSize,
                                        double learningRate,
                                        double cosineWeight) {
  return fitBatch(observations, previousPlans, targets, gradientSteps,
                  batchSize, learningRate, cosineWeight, false);
}

double GenerativePredictiveControl::fitAdamW(
    const Matrix& observations, const Matrix& previousPlans,
    const Matrix& targets, size_t gradientSteps, size_t batchSize,
    double learningRate, double cosineWeight) {
  return fitBatch(observations, previousPlans, targets, gradientSteps,
                  batchSize, learningRate, cosineWeight, true);
}

double GenerativePredictiveControl::fitBatch(
    const Matrix& observations, const Matrix& previousPlans,
    const Matrix& targets, size_t gradientSteps, size_t batchSize,
    double learningRate, double cosineWeight, bool useAdamW) {
  if (observations.rows() != observationDim_ ||
      targets.rows() != outputWeights_.rows() ||
      previousPlans.rows() != targets.rows() ||
      previousPlans.cols() != targets.cols() || !targets.cols() ||
      observations.cols() != targets.cols() || !observations.allFinite() ||
      !targets.allFinite() || !previousPlans.allFinite())
    throw std::invalid_argument(
        "Invalid observation and control-sequence training matrices.");
  if (!gradientSteps || !batchSize || !(learningRate > 0) ||
      !std::isfinite(learningRate) || !(cosineWeight >= 0) ||
      !std::isfinite(cosineWeight))
    throw std::invalid_argument("Invalid GPC training parameters.");
  std::uniform_int_distribution<Eigen::Index> example(0, targets.cols() - 1);
  std::uniform_real_distribution<double> time(0, 1);
  checkedProduct(std::max(inputWeights_.rows(), inputWeights_.cols()),
                 batchSize);
  const Eigen::Index columns = static_cast<Eigen::Index>(batchSize);
  Matrix inputs(inputWeights_.cols(), columns);
  Matrix velocity(targets.rows(), columns);
  Eigen::RowVectorXd weights(columns);
  const double rate = learningRate / static_cast<double>(batchSize);
  double loss = 0.;
  for (size_t step = 0; step < gradientSteps; ++step) {
    for (Eigen::Index j = 0; j < columns; ++j) {
      const auto k = example(generator_);
      const double t = time(generator_);
      const Vector source = normalDraw(targets.rows(), generator_);
      velocity.col(j) = targets.col(k) - source;
      const Vector improvement = targets.col(k) - previousPlans.col(k);
      const double denominator = improvement.norm() * velocity.col(j).norm();
      const double alignment =
          denominator > 0.
              ? std::clamp(improvement.dot(velocity.col(j)) / denominator, -1.,
                           1.)
              : 0.;
      weights[j] = std::exp(-cosineWeight * (1. - alignment));
      inputs.col(j) << source + t * velocity.col(j), observations.col(k), t;
    }
    // Evaluate and differentiate all examples at the same network parameters.
    const Matrix hidden =
        ((inputWeights_ * inputs).colwise() + hiddenBias_).array().tanh();
    const Matrix residual =
        ((outputWeights_ * hidden).colwise() + outputBias_) - velocity;
    loss = weights.dot(residual.colwise().squaredNorm());
    const Matrix out = (2. * residual.array()).rowwise() * weights.array();
    const Matrix back = (outputWeights_.transpose() * out).array() *
                        (1. - hidden.array().square());
    const Matrix gradIn = back * inputs.transpose();
    const Matrix gradOut = out * hidden.transpose();
    const Vector gradHidden = back.rowwise().sum(),
                 gradBias = out.rowwise().sum();
    if (!std::isfinite(loss) || !gradIn.allFinite() || !gradOut.allFinite() ||
        !gradHidden.allFinite() || !gradBias.allFinite())
      throw std::domain_error("GPC flow gradient diverged.");
    if (useAdamW) {
      internal::adamUpdate(firstMoment_, secondMoment_, adamUpdates_,
                           {{inputWeights_, gradIn},
                            {outputWeights_, gradOut},
                            {hiddenBias_, gradHidden},
                            {outputBias_, gradBias}},
                           learningRate, static_cast<double>(batchSize), 1e-4);
    } else {
      inputWeights_ -= rate * gradIn;
      outputWeights_ -= rate * gradOut;
      hiddenBias_ -= rate * gradHidden;
      outputBias_ -= rate * gradBias;
    }
  }
  return loss / static_cast<double>(batchSize);
}

double GenerativePredictiveControl::trainCycle(
    const Matrix& initialStates, const GPCTrainingParams& p,
    const std::function<Vector(const Vector&)>& observe,
    const std::function<Vector(const Vector&, const Vector&)>& advance,
    const std::function<Vector(const Vector&, const Matrix&)>& scoreBatch,
    const std::function<Vector(const Vector&)>& weights) {
  if (!initialStates.rows() || !initialStates.allFinite() || !observe ||
      !advance || !scoreBatch || !weights)
    throw std::invalid_argument(
        "GPC training needs finite initial states and simulation callbacks.");
  const Eigen::Index count =
      checkedProduct(initialStates.cols(), p.episodeLength);
  const size_t candidatesPerStep =
      sampleCount(p.gaussianSamples, p.policySamples);
  Matrix observations(observationDim_, count),
      previousPlans(outputWeights_.rows(), count),
      targets(outputWeights_.rows(), count);
  Eigen::Index k = 0;
  for (Eigen::Index e = 0; e < initialStates.cols(); ++e) {
    Vector x = initialStates.col(e);
    Vector seed =
        p.explorationStddev * normalDraw(outputWeights_.rows(), generator_);
    Matrix nominal =
        Eigen::Map<const Matrix>(seed.data(), controlDim_, horizon_);
    for (size_t step = 0; step < p.episodeLength; ++step, ++k) {
      const Vector y = observe(x);
      const Matrix batch =
          candidates(y, nominal, p.gaussianSamples, p.policySamples,
                     p.explorationStddev, p.warmStart, p.flowSteps);
      const Vector costs = scoreBatch(x, batch);
      if (costs.size() != static_cast<Eigen::Index>(candidatesPerStep))
        throw std::invalid_argument("GPC requires one cost per candidate.");
      const Vector w = weights(costs);
      if (w.size() != costs.size() || !w.allFinite() || (w.array() < 0).any() ||
          std::abs(w.sum() - 1.) > 1e-8)
        throw std::invalid_argument(
            "GPC requires normalized nonnegative search weights.");
      observations.col(k) = y;
      previousPlans.col(k) =
          Eigen::Map<const Vector>(nominal.data(), nominal.size());
      targets.col(k).noalias() =
          Eigen::Map<const Matrix>(batch.data(), nominal.size(), w.size()) * w;
      nominal = Eigen::Map<const Matrix>(targets.col(k).data(), controlDim_,
                                         horizon_);
      x = advance(x, nominal.col(0));
      if (x.size() != initialStates.rows() || !x.allFinite())
        throw std::domain_error("GPC simulation produced an invalid state.");
    }
  }
  return fitBatch(observations, previousPlans, targets, p.gradientSteps,
                  p.batchSize, p.learningRate, p.cosineWeight, p.useAdamW);
}

Vector GenerativePredictiveControl::riskCosts(const Matrix& costs,
                                              double tailFraction) {
  if (!costs.cols() || !(tailFraction >= 0 && tailFraction <= 1) ||
      !costs.allFinite())
    throw std::invalid_argument(
        "GPC domain risks require finite costs and a fraction in [0,1].");
  const double mass = tailFraction * static_cast<double>(costs.cols());
  Vector result(costs.rows());
  for (Eigen::Index i = 0; i < costs.rows(); ++i) {
    Vector sorted = costs.row(i).transpose();
    std::sort(sorted.data(), sorted.data() + sorted.size(),
              std::greater<double>());
    if (mass <= 1.) {
      result[i] = sorted[0];
      continue;
    }
    const Eigen::Index whole = static_cast<Eigen::Index>(std::floor(mass));
    double total = sorted.head(whole).sum();
    if (whole < sorted.size())
      total += (mass - static_cast<double>(whole)) * sorted[whole];
    result[i] = total / mass;
  }
  return result;
}

GPCProposal::GPCProposal(std::shared_ptr<GenerativePredictiveControl> policy,
                         const Vector& observation, const Matrix& covariance,
                         size_t gaussianSamples, size_t policySamples,
                         double warmStart, size_t flowSteps, size_t seed)
    : PolicyAugmentedProposal(
          covariance, gaussianSamples, policySamples,
          [policy, observation, warmStart, flowSteps](
              const Values&, const Matrix& previous, std::uint64_t sampleSeed) {
            std::mt19937_64 generator(sampleSeed);
            return policy->generateWithRng(observation, previous, warmStart,
                                           flowSteps, generator);
          },
          seed) {
  if (!policy ||
      covariance.rows() != static_cast<Eigen::Index>(policy->controlDim()))
    throw std::invalid_argument(
        "GPC+ proposal covariance must match the policy controls.");
  checkFlow(warmStart, flowSteps);
}

}  // namespace gtsam
