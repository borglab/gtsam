/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GradientsTorchExample.cpp
 * @brief Example of using GTSAM's Gradients class to train an MLP with GTSAM.
 * @author Frank Dellaert
 */

// Train a 1-8-2 odometry-correction MLP through GTSAM using LibTorch.
// Tutorial: python/gtsam/examples/nonlinear/GradientsPyTorchExample.ipynb.
// Build instructions: examples/README.md (set Torch_DIR if needed).

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <torch/csrc/autograd/autograd.h>
#include <torch/nn/modules/activation.h>
#include <torch/nn/modules/container/sequential.h>
#include <torch/nn/modules/linear.h>
#include <torch/nn/utils/convert_parameters.h>
#include <torch/optim/adam.h>
#include <torch/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

using namespace gtsam;
using torch::nn::utils::parameters_to_vector;

namespace gradients_torch {

constexpr size_t kSteps = 8;
constexpr double kPenaltyWeight = 1e-4;
const ParameterTag kMlpWeightsTag("odometry MLP");
struct Trajectory {
  std::array<double, kSteps> features;
  std::array<Point2, kSteps> measurements;
  std::array<Point2, kSteps + 1> truth;
  std::array<Point2, 2> anchors;
};

// Generate biased odometry and reference positions for the training loss.
Trajectory makeTrajectory(double phase, std::mt19937& rng) {
  std::normal_distribution<double> noise;
  Trajectory result;
  result.truth[0] = Point2::Zero();
  for (size_t i = 0; i < kSteps; ++i) {
    const double angle = 0.8 * i + phase, feature = std::sin(angle);
    const Point2 displacement{0.45 + 0.08 * std::cos(angle),
                              0.16 * std::sin(angle)};
    const Point2 bias{0.12 + 0.18 * std::sin(1.4 * feature),
                      -0.08 + 0.14 * feature * feature};
    result.features[i] = feature;
    result.truth[i + 1] = result.truth[i] + displacement;
    result.measurements[i] =
        displacement + bias + Point2{0.005 * noise(rng), 0.005 * noise(rng)};
  }
  // Only the two endpoint measurements become position priors.
  for (size_t i = 0; i < 2; ++i)
    result.anchors[i] =
        result.truth[i * kSteps] + Point2{0.01 * noise(rng), 0.01 * noise(rng)};
  return result;
}

// Copy an Eigen vector into a CPU tensor of doubles.
torch::Tensor toTensor(const Vector& values) {
  auto result = torch::empty({values.size()}, torch::kFloat64);
  std::copy_n(values.data(), values.size(), result.data_ptr<double>());
  return result;
}

// Return an owning Eigen vector, independent of Torch's storage.
Vector toVector(const torch::Tensor& values) {
  const auto contiguous = values.detach().contiguous();
  return Eigen::Map<const Vector>(contiguous.data_ptr<double>(),
                                  contiguous.numel());
}

// Keep the shared network fixed throughout each position solve and VJP query.
class NeuralOdometryFactor : public NoiseModelFactorT<Vector2, Point2, Point2> {
  using Base = NoiseModelFactorT<Vector2, Point2, Point2>;

 public:
  using Base::evaluateError;

  // Share the MLP and tag, then cache the corrected measurement.
  NeuralOdometryFactor(Key first, Key second, const Point2& measurement,
                       double feature, const SharedNoiseModel& noise,
                       const torch::nn::Sequential& model)
      : Base(noise, first, second),
        model_(model.ptr()),
        feature_(torch::tensor({feature}, torch::kFloat64)) {
    // Cache the correction without recording a Torch derivative graph.
    torch::NoGradGuard guard;
    correctedMeasurement_ = measurement - toVector(model_->forward(feature_));
  }

  // Compute the raw factor residual and its two position Jacobians.
  Vector2 evaluateError(const Point2& first, const Point2& second,
                        OptionalMatrixType firstJacobian,
                        OptionalMatrixType secondJacobian) const override {
    if (firstJacobian) *firstJacobian = -Matrix2::Identity();
    if (secondJacobian) *secondJacobian = Matrix2::Identity();
    // Return the raw factor residual; GTSAM applies the noise model.
    return second - first - correctedMeasurement_;
  }

  // Return this factor's contribution to the requested parameter gradient.
  std::optional<Vector> parameterVjp(
      const Values&, const ParameterTag& tag,
      const Vector& factorResidualCotangent) const override {
    if (tag != kMlpWeightsTag) return std::nullopt;
    // Only the MLP term of the raw factor residual depends on the weights.
    // Gradients has already accounted for noise weighting in this cotangent.
    const auto parts =
        torch::autograd::grad({model_->forward(feature_)}, model_->parameters(),
                              {toTensor(factorResidualCotangent)});
    // Preserve the model's parameter order when flattening the derivatives.
    return toVector(parameters_to_vector(parts));
  }

 private:
  std::shared_ptr<torch::nn::SequentialImpl> model_;
  torch::Tensor feature_;
  Point2 correctedMeasurement_;
};

struct SolvedBatch {
  NonlinearFactorGraph graph;
  Values solution;
  std::map<Key, Point2> targets;
};

// Build and solve a batch of paths that share the MLP's parameter tag.
SolvedBatch solveBatch(const std::vector<Trajectory>& trajectories,
                       const torch::nn::Sequential& model) {
  SolvedBatch batch;
  Values initial;
  const auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector2{0.08, 0.12});
  const auto startNoise = noiseModel::Isotropic::Sigma(2, 0.03);
  const auto endNoise = noiseModel::Isotropic::Sigma(2, 0.25);
  for (size_t path = 0; path < trajectories.size(); ++path) {
    const auto& trajectory = trajectories[path];
    std::array<Key, kSteps + 1> keys;
    Point2 integrated = trajectory.anchors[0];
    // Initialize positions by integrating the measured displacements.
    for (size_t i = 0; i <= kSteps; ++i) {
      keys[i] = Symbol('x', path * (kSteps + 1) + i);
      initial.insert(keys[i], integrated);
      if (i > 0) batch.targets.emplace(keys[i], trajectory.truth[i]);
      if (i < kSteps) integrated += trajectory.measurements[i];
    }
    batch.graph.emplace_shared<PriorFactor<Point2>>(
        keys.front(), trajectory.anchors[0], startNoise);
    for (size_t i = 0; i < kSteps; ++i)
      batch.graph.emplace_shared<NeuralOdometryFactor>(
          keys[i], keys[i + 1], trajectory.measurements[i],
          trajectory.features[i], odometryNoise, model);
    batch.graph.emplace_shared<PriorFactor<Point2>>(
        keys.back(), trajectory.anchors[1], endNoise);
  }
  batch.solution = GaussNewtonOptimizer(batch.graph, initial).optimize();
  return batch;
}

// Compute mean position loss and its derivatives on the optimized states.
std::pair<double, VectorValues> positionLoss(const SolvedBatch& batch) {
  double loss = 0.0;
  VectorValues cotangents;
  const double scale = 1.0 / batch.targets.size();
  for (const auto& [key, target] : batch.targets) {
    const Point2 error = batch.solution.at<Point2>(key) - target;
    loss += 0.5 * scale * error.squaredNorm();
    // Associate dL/dx with the corresponding position's key.
    cotangents.insert(key, scale * error);
  }
  return {loss, std::move(cotangents)};
}

// Return the L2 penalty on all MLP weights and biases.
torch::Tensor weightPenalty(const std::vector<torch::Tensor>& weights) {
  return 0.5 * kPenaltyWeight * parameters_to_vector(weights).square().sum();
}

// Train the MLP with Adam and report training and validation losses.
void train(const std::vector<Trajectory>& training,
           const std::vector<Trajectory>& validation,
           const torch::nn::Sequential& model) {
  auto weights = model->parameters();
  // Record tensor sizes to split GTSAM's flat parameter gradient.
  std::vector<int64_t> sizes;
  for (const auto& value : weights) sizes.push_back(value.numel());
  torch::optim::Adam optimizer(weights, torch::optim::AdamOptions(0.03));
  for (size_t epoch = 1; epoch <= 120; ++epoch) {
    optimizer.zero_grad();
    // Rebuild cached factor predictions at the current network weights.
    const auto batch = solveBatch(training, model);
    const Gradients gradients(batch.graph, batch.solution);
    const auto cotangents = positionLoss(batch).second;
    const Vector indirect =
        gradients.parameterGradient(cotangents, kMlpWeightsTag).value();
    // Torch supplies the direct loss term; the factor VJPs do not fill .grad.
    weightPenalty(weights).backward();
    {
      torch::NoGradGuard guard;
      // Add the derivative through GTSAM to each tensor's penalty derivative.
      const auto blocks = toTensor(indirect).split_with_sizes(sizes);
      for (size_t i = 0; i < weights.size(); ++i)
        weights[i].mutable_grad().add_(blocks[i].view_as(weights[i]));
    }
    // Update weights after the gradient queries for this training step.
    optimizer.step();
    if (epoch % 20 == 0) {
      const double trainingLoss =
          positionLoss(solveBatch(training, model)).first;
      // Validation reports position error without contributing to the update.
      const double validationLoss =
          positionLoss(solveBatch(validation, model)).first;
      std::cout << "Update " << epoch << ": training " << trainingLoss
                << ", held-out " << validationLoss << '\n';
    }
  }
}

}  // namespace gradients_torch

// Train an MLP on synthetic odometry paths and report position losses.
int main() {
  // 0. Initialize random number generators for reproducibility.
  using namespace gradients_torch;
  torch::manual_seed(7);
  std::mt19937 rng(7);

  // 1. Generate synthetic odometry paths with known ground truth.
  // Keep the validation paths separate from the training updates.
  std::vector<Trajectory> training, validation;
  for (double phase : {0.0, 0.8, 1.6, 2.4})
    training.push_back(makeTrajectory(phase, rng));
  for (double phase : {0.4, 2.0})
    validation.push_back(makeTrajectory(phase, rng));

  // 2. Build a small MLP to correct the biased odometry displacements.
  torch::nn::Sequential model(torch::nn::Linear(1, 8), torch::nn::Tanh(),
                              torch::nn::Linear(8, 2));
  // Match the precision of GTSAM's position and Jacobian calculations.
  model->to(torch::kFloat64);
  {
    // Start with small corrections to the measured displacements.
    torch::NoGradGuard guard;
    auto& output = model->at<torch::nn::LinearImpl>(2);
    output.weight.mul_(0.1);
    output.bias.zero_();
  }

  // 3. Calculate the initial position loss for both training and validation.
  const double initialLoss = positionLoss(solveBatch(training, model)).first;
  const double initialValidation =
      positionLoss(solveBatch(validation, model)).first;

  // 4. Train the MLP using the training paths.
  train(training, validation, model);

  // 5. Report the final position losses after training.
  const double finalLoss = positionLoss(solveBatch(training, model)).first;
  const double finalValidation =
      positionLoss(solveBatch(validation, model)).first;
  std::cout << "Training position loss: " << initialLoss << " -> " << finalLoss
            << "\nHeld-out position loss: " << initialValidation << " -> "
            << finalValidation << '\n';

  return 0;
}
