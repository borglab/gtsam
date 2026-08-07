/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/** @file FastSync.cpp */

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/FastSync.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <stdexcept>

namespace gtsam {
namespace internal {

/* ************************************************************************* */
double FastSyncIsotropicSigma(const SharedNoiseModel& model) {
  if (!model) {
    throw std::invalid_argument("fastSync requires a noise model");
  }
  if (std::dynamic_pointer_cast<noiseModel::Robust>(model)) {
    throw std::invalid_argument(
        "fastSync does not support robust noise models");
  }
  if (model->isConstrained()) {
    throw std::invalid_argument(
        "fastSync does not support constrained noise models");
  }
  const auto diagonal = std::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonal) {
    throw std::invalid_argument(
        "fastSync requires a diagonal isotropic noise model");
  }

  const Vector sigmas = diagonal->sigmas();
  if (sigmas.size() == 0) {
    throw std::invalid_argument("fastSync noise model is empty");
  }
  const double sigma = sigmas(0);
  if (!std::isfinite(sigma) || sigma <= 0.0) {
    throw std::invalid_argument(
        "fastSync requires finite, positive noise sigmas");
  }
  const double tolerance = 1e-12 * std::max(1.0, std::abs(sigma));
  for (Eigen::Index i = 1; i < sigmas.size(); ++i) {
    if (!std::isfinite(sigmas(i)) || sigmas(i) <= 0.0 ||
        std::abs(sigmas(i) - sigma) > tolerance) {
      throw std::invalid_argument("fastSync requires an isotropic noise model");
    }
  }
  return sigma;
}

/* ************************************************************************* */
static void CheckConnected(
    const std::vector<FastSyncMeasurement>& measurements) {
  if (measurements.empty()) {
    throw std::invalid_argument(
        "fastSync requires at least one between measurement");
  }

  FastMap<Key, std::vector<Key>> adjacency;
  for (const auto& measurement : measurements) {
    adjacency[measurement.key1].push_back(measurement.key2);
    adjacency[measurement.key2].push_back(measurement.key1);
  }

  std::set<Key> visited;
  std::queue<Key> frontier;
  frontier.push(adjacency.begin()->first);
  visited.insert(adjacency.begin()->first);
  while (!frontier.empty()) {
    const Key key = frontier.front();
    frontier.pop();
    for (const Key neighbor : adjacency.at(key)) {
      if (visited.insert(neighbor).second) frontier.push(neighbor);
    }
  }
  if (visited.size() != adjacency.size()) {
    throw std::invalid_argument(
        "fastSync requires a connected measurement graph");
  }
}

/* ************************************************************************* */
FastMap<Key, Matrix> FastSyncSolveMatrices(
    const std::vector<FastSyncMeasurement>& measurements,
    const size_t matrixDimension) {
  CheckConnected(measurements);
  if (matrixDimension == 0) {
    throw std::invalid_argument("fastSync matrix dimension must be positive");
  }

  const Matrix identity = Matrix::Identity(matrixDimension, matrixDimension);
  const Vector zero = Vector::Zero(matrixDimension);
  GaussianFactorGraph reducedGraph;
  for (const auto& measurement : measurements) {
    if (!std::isfinite(measurement.sigma) || measurement.sigma <= 0.0) {
      throw std::invalid_argument(
          "fastSync requires finite, positive measurement sigmas");
    }
    if (measurement.measured.rows() !=
            static_cast<Eigen::Index>(matrixDimension) ||
        measurement.measured.cols() !=
            static_cast<Eigen::Index>(matrixDimension)) {
      throw std::invalid_argument(
          "fastSync measurement matrix has the wrong dimension");
    }
    reducedGraph.add(
        measurement.key1, -measurement.measured.transpose(), measurement.key2,
        identity, zero,
        noiseModel::Isotropic::Sigma(matrixDimension, measurement.sigma));
  }

  const Ordering ordering = Ordering::Metis(reducedGraph);
  if (ordering.empty()) {
    throw std::runtime_error("fastSync METIS ordering failed");
  }
  const Key gaugeKey = ordering.back();
  reducedGraph.add(gaugeKey, identity, zero,
                   noiseModel::Unit::Create(matrixDimension));

  const auto bayesNet = reducedGraph.eliminateSequential(ordering, EliminateQR);
  if (!bayesNet || bayesNet->size() != ordering.size()) {
    throw std::runtime_error("fastSync sequential QR elimination failed");
  }

  FastMap<Key, Matrix> solution;
  solution.emplace(gaugeKey, identity);
  for (size_t reverseIndex = bayesNet->size(); reverseIndex > 0;
       --reverseIndex) {
    const auto& conditional = bayesNet->at(reverseIndex - 1);
    const Key frontalKey = conditional->firstFrontalKey();
    if (frontalKey == gaugeKey) continue;
    if (conditional->nrFrontals() != 1) {
      throw std::runtime_error(
          "fastSync expected one frontal variable per conditional");
    }

    Matrix sum = Matrix::Zero(matrixDimension, matrixDimension);
    size_t parentIndex = 0;
    for (const Key parentKey : conditional->parents()) {
      const auto parent = solution.find(parentKey);
      if (parent == solution.end()) {
        throw std::runtime_error(
            "fastSync encountered an unsolved separator variable");
      }
      const Matrix parentBlock = conditional->S().block(
          0, parentIndex * matrixDimension, matrixDimension, matrixDimension);
      sum.noalias() += parent->second * parentBlock.transpose();
      ++parentIndex;
    }

    const Matrix R = conditional->R();
    const Matrix transposeEstimate =
        -R.triangularView<Eigen::Upper>().solve(sum.transpose());
    solution.emplace(frontalKey, transposeEstimate.transpose());
  }

  if (solution.size() != ordering.size()) {
    throw std::runtime_error("fastSync block back-substitution failed");
  }
  return solution;
}

}  // namespace internal
}  // namespace gtsam
