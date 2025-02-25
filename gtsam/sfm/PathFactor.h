/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PathFactor.h
 * @date February 2025
 * @author Akshay Krishnan and Frank Dellaert
 * @brief Product of transforms factor
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <stdexcept>
#include <vector>

namespace gtsam {

/**
 * \brief PathFactor
 *
 * A NonlinearFactor defined on a sequence (path) of relative rotations (or,
 * more generally, Lie group elements) whose predicted overall transformation is
 * given by the composition along the path.
 *
 * Template parameter G is assumed to be a LieGroup derivative.
 */
template <class G>
class PathFactor : public NoiseModelFactor {
 public:
  typedef std::shared_ptr<PathFactor> shared_ptr;

  // _d is a static constexpr member that specifies the dimension of G
  static const Matrix Id_;

 private:
  std::uint32_t i_, j_;
  /// Measured overall relative transformation along the path.
  G measured_;
  /// Ordered sequence of EdgeKeys forming the path.
  std::vector<EdgeKey> path_;

 public:
  /**
   * \brief Constructor.
   * \param i,j: the start and end of the path.
   * \param measured The measured overall transformation.
   * \param path Vector of EdgeKeys representing a path from i to j.
   *
   * \note The edges are the ones in the tree, and might be reversed. For
   * example, to predict R14, we might have edge keys (1,2),(3,2),(3,4).
   * In this case, the measurement for the middle key will be inverted.
   */
  PathFactor(std::uint32_t i, std::uint32_t j, const G& measured,
             const std::vector<EdgeKey>& path,
             const SharedNoiseModel& noiseModel = nullptr)
      : NoiseModelFactor(noiseModel, KeysfromPath(path)),
        i_(i),
        j_(j),
        measured_(measured),
        path_(path) {
    CheckPath(i, j, path);
  }

  /// Return the factor dimension (Lie algebra dimension of G).
  size_t dim() const override { return G::dimension; }

 private:
  /// Helper: Check that the path is valid for given (i,j) prediction.
  static void CheckPath(std::uint32_t i, std::uint32_t j,
                        const std::vector<EdgeKey>& path) {
    if (path.empty()) {
      throw std::invalid_argument("Path is empty");
    }

    std::uint32_t current = i;
    for (const auto& kl : path) {
      if (kl.i() == current) {
        current = kl.j();
      } else if (kl.j() == current) {
        current = kl.i();
      } else {
        throw std::invalid_argument(
            "Path is not continuous at edge starting with " +
            std::to_string(kl.i()));
      }
    }

    if (current != j) {
      throw std::invalid_argument("Path does not end at j");
    }
  }

  /// Helper: Extract Keys from a vector of EdgeKey.
  static KeyVector KeysfromPath(const std::vector<EdgeKey>& path) {
    KeyVector keys;
    for (const auto& ek : path) {
      keys.push_back(static_cast<Key>(ek));
    }
    return keys;
  }

  /**
   * \brief Evaluate the error.
   */
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    const size_t n = path_.size();
    std::vector<std::pair<G, Matrix>> Qs;
    G T_jk = G::Identity();
    size_t i = 0;
    std::uint32_t current = j_;  // Start from the end node.
    for (auto it = path_.rbegin(); it != path_.rend(); ++it) {
      const auto& kl = *it;
      const G G_kl = values.at<G>(kl);
      G G_k;
      Matrix localDerivative;
      if (kl.j() == current) {
        G_k = G_kl.inverse();
        current = kl.i();
        localDerivative = Id_;
        if (H) Qs.emplace_back(T_jk, Id_);
      } else {
        assert(kl.i() == current);
        G_k = G_kl;
        current = kl.j();
        localDerivative = -G_kl.AdjointMap();
        if (H) Qs.emplace_back(T_jk, -G_kl.AdjointMap());
      }
      if (H) H->at(n - 1 - i) = T_jk.AdjointMap() * localDerivative;
      T_jk = T_jk * G_k;
      i += 1;
    }
    // The overall effective transform is the inverse of T_jk.
    G T_ij = T_jk.inverse();
    G residual = measured_.between(T_ij);

    if (H) {
      Matrix DLog;
      Vector b = G::Logmap(residual, DLog);
      // Compute the Jacobians in forward order.
      for (size_t i = 0; i < n; i++) {
        H->at(i) = DLog * H->at(i);
      }
      return b;
    } else {
      return G::Logmap(residual);
    }
  }

  /// Clone the factor.
  std::shared_ptr<NonlinearFactor> clone() const override {
    return std::make_shared<PathFactor<G>>(*this);
  }
};

template <class G>
const Matrix PathFactor<G>::Id_ = Matrix::Identity(G::dimension, G::dimension);

}  // namespace gtsam
