/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <gtsam/nonlinear/Gradients.h>

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {
namespace {

std::invalid_argument factorError(size_t index, const std::string& message) {
  return std::invalid_argument("Gradients factor " + std::to_string(index) +
                               ": " + message);
}

noiseModel::Gaussian::shared_ptr checkNoise(const SharedNoiseModel& model,
                                          size_t rows, size_t index) {
  if (!model) return {};
  const auto gaussian = std::dynamic_pointer_cast<noiseModel::Gaussian>(model);
  if (!gaussian || model->isConstrained() || model->dim() != rows)
    throw factorError(
        index, "requires dimension-matched unconstrained Gaussian noise");
  if (const auto diagonal =
          std::dynamic_pointer_cast<noiseModel::Diagonal>(model)) {
    if (!diagonal->sigmasRef().allFinite() ||
        (diagonal->sigmasRef().array() <= 0).any())
      throw factorError(index, "noise sigmas must be finite and positive");
  } else {
    const Matrix whitening = gaussian->R();
    if (!whitening.allFinite() || (whitening.diagonal().array() == 0).any())
      throw factorError(index,
                        "noise must have finite positive-definite covariance");
  }
  return gaussian;
}

JacobianFactor::shared_ptr linearizeFactorResidual(
    const NonlinearFactor& factor, const Values& solution, size_t index) {
  const auto* residualFactor = dynamic_cast<const NoiseModelFactor*>(&factor);
  if (!residualFactor)
    throw factorError(
        index, "requires factor residual Jacobians from a NoiseModelFactor");
  std::vector<Matrix> blocks(factor.size());
  const Vector residual = residualFactor->unwhitenedError(solution, blocks);
  if (blocks.size() != factor.size())
    throw factorError(index, "factor residual Jacobian block count mismatch");
  if (!residual.allFinite())
    throw factorError(index, "nonfinite factor residual");
  const auto noise =
      checkNoise(residualFactor->noiseModel(), residual.size(), index);
  std::vector<std::pair<Key, Matrix>> terms;
  KeySet seen;
  for (size_t slot = 0; slot < factor.size(); ++slot) {
    const Key key = factor.keys()[slot];
    if (!seen.insert(key).second || blocks[slot].rows() != residual.size() ||
        !solution.exists(key) ||
        blocks[slot].cols() != static_cast<DenseIndex>(solution.at(key).dim()))
      throw factorError(index,
                        "factor residual Jacobian/key dimension mismatch");
    if (noise) noise->WhitenInPlace(blocks[slot]);
    if (!blocks[slot].allFinite())
      throw factorError(index, "nonfinite whitened Jacobian");
    terms.emplace_back(key, std::move(blocks[slot]));
  }
  return std::make_shared<JacobianFactor>(terms, Vector::Zero(residual.size()));
}

GaussianFactorGraph linearizeFactorResiduals(const NonlinearFactorGraph& graph,
                                           const Values& solution) {
  GaussianFactorGraph result;
  result.reserve(graph.size());
  // Zero-row factors preserve empty slots for native graph products.
  for (size_t index = 0; index < graph.size(); ++index) {
    const auto& factor = graph[index];
    result.push_back(factor && factor->active(solution)
                         ? linearizeFactorResidual(*factor, solution, index)
                         : std::make_shared<JacobianFactor>(Vector{}));
  }
  return result;
}

void recordDimensions(const JacobianFactor& factor, size_t index,
                      VectorValues::Dims* dimensions) {
  KeySet seen;
  for (auto key = factor.begin(); key != factor.end(); ++key) {
    const auto dimension = factor.getDim(key);
    if (dimension <= 0 || !seen.insert(*key).second)
      throw factorError(index, "repeated key or nonpositive tangent dimension");
    const auto [entry, inserted] = dimensions->emplace(*key, dimension);
    if (!inserted && entry->second != static_cast<size_t>(dimension))
      throw factorError(index, "inconsistent tangent dimension at " +
                                   DefaultKeyFormatter(*key));
  }
}

GaussianFactorGraph whitenFactorResiduals(const GaussianFactorGraph& graph) {
  GaussianFactorGraph result;
  result.reserve(graph.size());
  // Zero-row factors preserve empty slots for native graph products.
  for (size_t index = 0; index < graph.size(); ++index) {
    if (!graph[index]) {
      result.push_back(std::make_shared<JacobianFactor>(Vector{}));
      continue;
    }
    const auto* factor =
        dynamic_cast<const JacobianFactor*>(graph[index].get());
    if (!factor)
      throw factorError(index, "requires explicit JacobianFactor rows");
    checkNoise(factor->get_model(), factor->rows(), index);
    if (!factor->getA().allFinite() || !factor->getb().allFinite())
      throw factorError(index, "nonfinite factor residual linearization");
    auto whitened = std::make_shared<JacobianFactor>(*factor);
    // Affine offsets do not enter derivative queries.
    whitened->getb().setZero();
    if (const auto noise = whitened->get_model()) {
      noise->WhitenInPlace(whitened->matrixObject().full());
      whitened->get_model().reset();
    }
    if (!whitened->getA().allFinite())
      throw factorError(index, "nonfinite whitened Jacobian");
    result.push_back(std::move(whitened));
  }
  return result;
}

GaussianFactorGraph::Eliminate eliminationFunction(
    Gradients::Factorization factorization) {
  if (factorization != Gradients::CHOLESKY && factorization != Gradients::QR)
    throw std::invalid_argument("Gradients: unknown factorization");
  GaussianFactorGraph::Eliminate eliminate = EliminatePreferCholesky;
  if (factorization == Gradients::QR) eliminate = EliminateQR;
  return [eliminate](const GaussianFactorGraph& factors,
                     const Ordering& frontals) {
    auto result = eliminate(factors, frontals);
    // Reject nonfinite coefficients and zero pivots during construction.
    if (!result.first->getA().allFinite() ||
        (result.first->R().diagonal().array() == 0).any())
      throw IndeterminateSystemException(result.first->firstFrontalKey());
    return result;
  };
}

}  // namespace

void Gradients::initializeDimensions() {
  for (size_t index = 0; index < graph_.size(); ++index)
    recordDimensions(static_cast<const JacobianFactor&>(*graph_[index]), index,
                     &dimensions_);
}

void Gradients::computeBayesTree(const std::optional<Ordering>& ordering,
                                Factorization factorization) {
  const auto eliminate = eliminationFunction(factorization);
  if (ordering) {
    KeySet seen;
    for (Key key : *ordering)
      if (!seen.insert(key).second || !dimensions_.count(key))
        throw std::invalid_argument(
            "Gradients: ordering must contain each key once");
    if (seen.size() != dimensions_.size())
      throw std::invalid_argument("Gradients: ordering omits keys");
  }
  if (dimensions_.empty()) {
    bayesTree_ = std::make_shared<GaussianBayesTree>();
    return;
  }
  GaussianFactorGraph active;
  for (const auto& factor : graph_)
    if (!factor->empty()) active.push_back(factor);
  bayesTree_ = ordering
                   ? active.eliminateMultifrontal(*ordering, eliminate)
                   : active.eliminateMultifrontal(Ordering::COLAMD, eliminate);
}

Gradients::Gradients(const NonlinearFactorGraph& graph, const Values& solution,
                    Factorization factorization)
    : Gradients(linearizeFactorResiduals(graph, solution), factorization,
                std::nullopt) {
  nonlinearGraph_ = graph;
  solution_ = solution;
}

Gradients::Gradients(const NonlinearFactorGraph& graph, const Values& solution,
                    const Ordering& ordering, Factorization factorization)
    : Gradients(linearizeFactorResiduals(graph, solution), factorization,
                ordering) {
  nonlinearGraph_ = graph;
  solution_ = solution;
}

Gradients::Gradients(const GaussianFactorGraph& graph,
                    Factorization factorization)
    : Gradients(whitenFactorResiduals(graph), factorization, std::nullopt) {}

Gradients::Gradients(const GaussianFactorGraph& graph, const Ordering& ordering,
                    Factorization factorization)
    : Gradients(whitenFactorResiduals(graph), factorization, ordering) {}

Gradients::Gradients(GaussianFactorGraph&& graph, Factorization factorization,
                    const std::optional<Ordering>& ordering)
    : graph_(std::move(graph)) {
  initializeDimensions();
  computeBayesTree(ordering, factorization);
}

Gradients::Gradients(const GaussianFactorGraph& linearizedGraph,
                    const GaussianBayesTree& bayesTree)
    : graph_(whitenFactorResiduals(linearizedGraph)),
      bayesTree_(std::make_shared<GaussianBayesTree>(bayesTree)) {
  initializeDimensions();
  // Tree copies own topology and share conditional and noise-model data.
  const GaussianFactorGraph conditionals(*bayesTree_);
  if (conditionals.getKeyDimMap() != dimensions_)
    throw std::invalid_argument(
        "Gradients: tree and factor residual graph tangent dimensions differ");
  for (size_t index = 0; index < conditionals.size(); ++index) {
    const auto& conditional =
        static_cast<const GaussianConditional&>(*conditionals[index]);
    checkNoise(conditional.get_model(), conditional.rows(), index);
    if (!conditional.getA().allFinite() ||
        conditional.R().rows() != conditional.R().cols() ||
        !conditional.R()
             .triangularView<Eigen::StrictlyLower>()
             .toDenseMatrix()
             .isZero(0))
      throw std::invalid_argument("Gradients: invalid imported conditional");
    if ((conditional.R().diagonal().array() == 0).any())
      throw IndeterminateSystemException(conditional.firstFrontalKey());
  }
}

void Gradients::print(const std::string& prefix) const {
  size_t rows = 0;
  for (const auto& factor : graph_)
    rows += static_cast<const JacobianFactor&>(*factor).rows();
  std::cout << prefix << dimensions_.size() << " state blocks, "
            << graph_.size() << " factor slots, " << rows
            << " factor residual rows\n";
}

VectorValues Gradients::adjoint(const VectorValues& stateCotangents) const {
  const VectorValues intermediate =
      bayesTree_->backSubstituteTranspose(stateCotangents);
  return bayesTree_->backSubstitute(intermediate);
}

std::optional<Vector> Gradients::parameterGradient(
    const VectorValues& stateCotangents, const ParameterTag& parameter) const {
  if (!solution_)
    throw std::logic_error(
        "Gradients::parameterGradient requires a nonlinear graph and solution");
  const Errors whitenedCotangents = pullback(stateCotangents);
  auto cotangent = whitenedCotangents.begin();
  std::optional<Vector> result;
  for (size_t index = 0; index < nonlinearGraph_.size(); ++index, ++cotangent) {
    const auto& factor = nonlinearGraph_[index];
    if (!factor || !factor->active(*solution_)) continue;
    const auto& noiseFactor = static_cast<const NoiseModelFactor&>(*factor);
    Vector factorResidualCotangent = *cotangent;
    // The cotangent conversion uses W transpose, not the inverse whitening.
    if (const auto& noise = noiseFactor.noiseModel()) {
      if (const auto diagonal =
              std::dynamic_pointer_cast<noiseModel::Diagonal>(noise)) {
        diagonal->whitenInPlace(factorResidualCotangent);
      } else {
        const auto gaussian =
            std::static_pointer_cast<noiseModel::Gaussian>(noise);
        factorResidualCotangent =
            gaussian->R().transpose() * factorResidualCotangent;
      }
    }
    if (!factorResidualCotangent.allFinite())
      throw std::runtime_error(
          "Gradients: nonfinite unwhitened factor residual cotangent");
    const auto contribution =
        noiseFactor.parameterVjp(*solution_, parameter, factorResidualCotangent);
    if (!contribution) continue;
    if (contribution->size() == 0 || !contribution->allFinite() ||
        (result && result->size() != contribution->size()))
      throw factorError(index, "invalid parameter VJP dimension or value");
    if (result)
      *result += *contribution;
    else
      result = *contribution;
    if (!result->allFinite())
      throw std::runtime_error("Gradients: nonfinite accumulated parameter gradient");
  }
  return result;
}

VectorValues Gradients::pushforward(
    const Errors& factorResidualDirection) const {
  if (factorResidualDirection.size() != graph_.size())
    throw std::invalid_argument(
        "Gradients: expected one factor residual block per factor slot");
  auto block = factorResidualDirection.begin();
  for (size_t index = 0; index < graph_.size(); ++index, ++block) {
    const auto& factor = static_cast<const JacobianFactor&>(*graph_[index]);
    if (block->size() != static_cast<DenseIndex>(factor.rows()) ||
        !block->allFinite())
      throw factorError(index,
                        "invalid factor residual direction dimension or value");
  }
  VectorValues rhs;
  graph_.transposeMultiplyAdd(-1, factorResidualDirection, rhs);
  return adjoint(rhs);
}

Errors Gradients::pullback(const VectorValues& stateCotangents) const {
  Errors result = graph_ * adjoint(stateCotangents);
  for (Vector& factorResidualCotangent : result) {
    factorResidualCotangent *= -1;
    if (!factorResidualCotangent.allFinite())
      throw std::runtime_error(
          "Gradients: nonfinite factor residual cotangents");
  }
  return result;
}

}  // namespace gtsam
