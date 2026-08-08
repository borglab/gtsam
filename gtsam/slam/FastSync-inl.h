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

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace gtsam {

/// Fixed-size FAST-Sync solver for the matrix Lie group T.
template <class T>
struct FastSyncSolver {
  using LieAlgebra = typename T::LieAlgebra;
  static constexpr int N = LieAlgebra::RowsAtCompileTime;
  static_assert(N != Eigen::Dynamic && N > 0,
                "FastSync requires a positive compile-time matrix dimension");
  static_assert(LieAlgebra::ColsAtCompileTime == N,
                "FastSync requires a square matrix representation");

  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorN = Eigen::Matrix<double, N, 1>;

  /// Extract factors, validate, and build the reduced graph.
  explicit FastSyncSolver(const NonlinearFactorGraph& graph) {
    for (const auto& factor : graph) {
      if (const auto between =
              std::dynamic_pointer_cast<BetweenFactor<T>>(factor)) {
        if (between->noiseModel()->dim() != T::dimension) {
          throw std::invalid_argument(
              "fastSync noise dimension must match the group dimension");
        }
        const double sigma = isotropicSigma(between->noiseModel());
        if (!std::isfinite(sigma) || sigma <= 0.0) {
          throw std::invalid_argument(
              "FastSync requires finite, positive measurement sigmas");
        }
        const Key k1 = between->key1(), k2 = between->key2();
        const MatrixN firstBlock = -between->measured().matrix().transpose();
        // Whitening by sigma gives the paper's precision kappa = 1 / sigma^2.
        reducedGraph_.emplace_shared<JacobianFactor>(
            k1, firstBlock, k2, MatrixN::Identity(), VectorN::Zero(),
            noiseModel::Isotropic::Sigma(N, sigma));
      } else if (const auto prior =
                     std::dynamic_pointer_cast<PriorFactor<T>>(factor)) {
        if (++priorCount_ > 1) {
          throw std::invalid_argument(
              "fastSync supports at most one matching prior");
        }
        priorKey_ = prior->key();
        priorValue_ = prior->prior();
      }
    }
    if (reducedGraph_.empty()) {
      throw std::invalid_argument(
          "FastSync requires at least one between measurement");
    }
  }

 private:
  size_t priorCount_ = 0;
  Key priorKey_ = 0;
  T priorValue_ = traits<T>::Identity();
  GaussianFactorGraph reducedGraph_;

  /// Extract the isotropic sigma from a noise model, or throw if not isotropic.
  static double isotropicSigma(const SharedNoiseModel& model) {
    const auto isotropic =
        std::dynamic_pointer_cast<noiseModel::Isotropic>(model);
    if (!isotropic) {
      throw std::invalid_argument("FastSync requires isotropic noise model");
    }
    return isotropic->sigma();
  }

  /// Back-substitute a single conditional, skipping the gauge variable.
  static void backSubstituteConditional(const GaussianConditional& conditional,
                                        const Key& gaugeKey, Values& solution) {
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
      sum.noalias() +=
          solution.at<MatrixN>(parentKey) * parentBlock.transpose();
      ++parentIndex;
    }

    const MatrixN R = conditional.R();
    const MatrixN transposeEstimate =
        -R.template triangularView<Eigen::Upper>().solve(sum.transpose());
    solution.insert(frontalKey, MatrixN(transposeEstimate.transpose()));
  }

 public:
  /// Solve the relaxed problem and return per-key ambient N×N matrices.
  Values solve() const {
    const MatrixN identity = MatrixN::Identity();
    const VectorN zero = VectorN::Zero();

    GaussianFactorGraph graph = reducedGraph_;
    const Ordering ordering = Ordering::Metis(graph);
    if (ordering.empty()) {
      throw std::runtime_error("FastSync METIS ordering failed");
    }
    const Key gaugeKey = ordering.back();
    graph.emplace_shared<JacobianFactor>(gaugeKey, identity, zero,
                                         noiseModel::Unit::Create(N));

    const auto bayesNet = graph.eliminateSequential(ordering, EliminateQR);
    if (!bayesNet || bayesNet->size() != ordering.size()) {
      throw std::runtime_error("FastSync sequential QR elimination failed");
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

  /// Project ambient matrices to T, align to the stored prior, and return.
  Values projectAndAlign(const Values& relaxed) const {
    Values projected;
    for (const Key key : relaxed.keys()) {
      projected.insert(
          key, FastSyncProjection<T>::project(relaxed.at<MatrixN>(key)));
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
};

/* ************************************************************************* */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph) {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  const FastSyncSolver<T> solver(graph);
  const Values relaxed = solver.solve();
  return solver.projectAndAlign(relaxed);
}

}  // namespace gtsam
