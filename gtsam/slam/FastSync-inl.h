/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FastSync-inl.h
 * @brief Template implementation for fixed-size FAST-Sync.
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace gtsam {

template <class T>
double FastSync<T>::isotropicSigma(const SharedNoiseModel& model) {
  const auto isotropic =
      std::dynamic_pointer_cast<noiseModel::Isotropic>(model);
  if (!isotropic) {
    throw std::invalid_argument("FastSync requires isotropic noise model");
  }
  return isotropic->sigma();
}

template <class T>
FastSync<T>::FastSync(const NonlinearFactorGraph& graph) {
  const auto addMeasurement = [this](Key key1, Key key2,
                                     const T& measurement,
                                     const SharedNoiseModel& model,
                                     size_t expectedNoiseDimension) {
    if (model->dim() != expectedNoiseDimension) {
      throw std::invalid_argument(
          "fastSync noise dimension does not match the factor residual");
    }
    const double sigma = isotropicSigma(model);
    if (!std::isfinite(sigma) || sigma <= 0.0) {
      throw std::invalid_argument(
          "FastSync requires finite, positive measurement sigmas");
    }
    const MatrixN firstBlock = -measurement.matrix().transpose();
    // Whitening by sigma gives the paper's precision kappa = 1 / sigma^2.
    reducedGraph_.emplace_shared<JacobianFactor>(
        key1, firstBlock, key2, MatrixN::Identity(), VectorN::Zero(),
        noiseModel::Isotropic::Sigma(N, sigma));
  };
  const auto addPrior = [this](Key key, const T& value) {
    if (++priorCount_ > 1) {
      throw std::invalid_argument(
          "fastSync supports at most one matching prior");
    }
    priorKey_ = key;
    priorValue_ = value;
  };

  for (const auto& factor : graph) {
    if (const auto between =
            std::dynamic_pointer_cast<BetweenFactor<T>>(factor)) {
      addMeasurement(between->key1(), between->key2(), between->measured(),
                     between->noiseModel(), T::dimension);
    } else if (const auto between =
                   std::dynamic_pointer_cast<FrobeniusBetweenFactor<T>>(
                       factor)) {
      addMeasurement(between->key1(), between->key2(), between->measured(),
                     between->noiseModel(), N * N);
    } else if (const auto prior =
                   std::dynamic_pointer_cast<PriorFactor<T>>(factor)) {
      addPrior(prior->key(), prior->prior());
    } else if (const auto prior =
                   std::dynamic_pointer_cast<FrobeniusPrior<T>>(factor)) {
      addPrior(prior->key(),
               FastSyncProjection<T>::project(prior->priorMatrix()));
    }
  }
  if (reducedGraph_.empty()) {
    throw std::invalid_argument(
        "FastSync requires at least one between measurement");
  }
}

template <class T>
void FastSync<T>::backSubstituteConditional(
    const GaussianConditional& conditional, const Key& gaugeKey,
    Values& solution) {
  const Key frontalKey = conditional.firstFrontalKey();
  if (frontalKey == gaugeKey) return;
  if (conditional.nrFrontals() != 1) {
    throw std::runtime_error(
        "FastSync expected one frontal variable per conditional");
  }

  // The conditional encodes the paper's block equation
  // X_j R_jj.transpose() + sum_k X_k R_jk.transpose() = 0.
  MatrixN sum = MatrixN::Zero();
  size_t parentIndex = 0;
  for (const Key parentKey : conditional.parents()) {
    if (!solution.exists(parentKey)) {
      throw std::runtime_error(
          "FastSync encountered an unsolved separator variable");
    }
    const MatrixN parentBlock = conditional.S().template block<N, N>(
        0, static_cast<Eigen::Index>(parentIndex * N));
    sum.noalias() += solution.at<MatrixN>(parentKey) * parentBlock.transpose();
    ++parentIndex;
  }

  const MatrixN R = conditional.R();
  const MatrixN transposeEstimate =
      -R.template triangularView<Eigen::Upper>().solve(sum.transpose());
  solution.insert(frontalKey, MatrixN(transposeEstimate.transpose()));
}

template <class T>
Values FastSync<T>::solve(Ordering::OrderingType orderingType) const {
  return solveOrdered(Ordering::Create(orderingType, reducedGraph_));
}

template <class T>
Values FastSync<T>::solve(const Ordering& ordering) const {
  const KeySet graphKeys = reducedGraph_.keys();
  const KeySet orderingKeys(ordering);
  if (ordering.size() != graphKeys.size() || orderingKeys != graphKeys) {
    throw std::invalid_argument(
        "FastSync ordering must contain every measurement graph key exactly "
        "once");
  }
  return solveOrdered(ordering);
}

template <class T>
Values FastSync<T>::solveOrdered(const Ordering& ordering) const {
  const MatrixN identity = MatrixN::Identity();
  const VectorN zero = VectorN::Zero();

  GaussianFactorGraph graph = reducedGraph_;
  const Key gaugeKey = ordering.back();
  graph.emplace_shared<JacobianFactor>(gaugeKey, identity, zero,
                                       noiseModel::Unit::Create(N));

  const auto bayesNet =
      graph.eliminateSequential(ordering, EliminatePreferCholesky);
  if (!bayesNet || bayesNet->size() != ordering.size()) {
    throw std::runtime_error("FastSync sequential Cholesky elimination failed");
  }

  Values solution;
  solution.insert(gaugeKey, identity);
  for (size_t reverseIndex = bayesNet->size(); reverseIndex > 0;
       --reverseIndex) {
    const auto& conditional = bayesNet->at(reverseIndex - 1);
    backSubstituteConditional(*conditional, gaugeKey, solution);
  }

  if (solution.size() != ordering.size()) {
    throw std::runtime_error("FastSync block back-substitution failed");
  }
  return solution;
}

template <class T>
Values FastSync<T>::projectAndAlign(const Values& relaxed) const {
  Values projected;
  for (const Key key : relaxed.keys()) {
    projected.insert(key,
                     FastSyncProjection<T>::project(relaxed.at<MatrixN>(key)));
  }

  if (priorCount_ == 0) return projected;
  if (!projected.exists(priorKey_)) {
    throw std::invalid_argument(
        "fastSync prior key is not in the measurement graph");
  }

  // Align with one common left transformation, preserving relative estimates.
  const T estimatedPrior = projected.at<T>(priorKey_);
  const T alignment =
      traits<T>::Compose(priorValue_, traits<T>::Inverse(estimatedPrior));
  Values aligned;
  for (const auto& keyValue : projected.extract<T>()) {
    aligned.insert(keyValue.first,
                   traits<T>::Compose(alignment, keyValue.second));
  }
  return aligned;
}

/* ************************************************************************* */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph,
                Ordering::OrderingType orderingType) {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  const FastSync<T> solver(graph);
  const Values relaxed = solver.solve(orderingType);
  return solver.projectAndAlign(relaxed);
}

/* ************************************************************************* */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph, const Ordering& ordering) {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  const FastSync<T> solver(graph);
  const Values relaxed = solver.solve(ordering);
  return solver.projectAndAlign(relaxed);
}

}  // namespace gtsam
