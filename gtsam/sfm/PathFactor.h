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
  /// We store inverse of measured transform g_ij.
  G measured_ji_;

  /// Ordered sequence of EdgeKeys forming the path, with orientation
  std::vector<std::pair<EdgeKey, bool>> path_;

 public:
  /**
   * \brief Constructor.
   * \param i,j: the start and end of the path.
   * \param g_ij The measured overall transformation.
   * \param path Vector of EdgeKeys representing a path from i to j.
   *
   * \note The edges are the ones in the tree, and might be reversed. For
   * example, to predict R14, we might have edge keys (1,2),(3,2),(3,4).
   * In this case, the measurement for the middle key will be inverted.
   */
  PathFactor(std::uint32_t i, std::uint32_t j, const G& g_ij,
             const std::vector<EdgeKey>& path,
             const SharedNoiseModel& noiseModel = nullptr)
      : NoiseModelFactor(noiseModel, KeysfromPath(path)),
        measured_ji_(g_ij.inverse()) {
    CheckAndSetPath(i, j, path);
  }

  /// Return the factor dimension (Lie algebra dimension of G).
  size_t dim() const override { return G::dimension; }

 private:
  /// Helper: Check that the path is valid and calculate orientations.
  void CheckAndSetPath(std::uint32_t i, std::uint32_t j,
                       const std::vector<EdgeKey>& originalPath) {
    if (originalPath.empty()) {
      throw std::invalid_argument("Path is empty");
    }

    std::uint32_t current = i;
    for (const auto& kl : originalPath) {
      if (kl.i() != current && kl.j() != current)
        throw std::invalid_argument("Path is not continuous at edge " +
                                    (std::string)kl);
      const bool alongPath = (kl.i() == current);
      current = alongPath ? kl.j() : kl.i();
      path_.emplace_back(kl, alongPath);
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
   * \brief Predict the measurement.
   */
  G predict(const Values& values, OptionalMatrixVecType H = nullptr) const {
    // Below we loop over the path in reverse order, because the Jacobians are
    // Adjoints of the reverse accumulated transform (partial) g_ji. After the
    // loop we take the inverse of g_ji to obtain the prediction g_ij.

    // Accumulate reverse transform from j to i
    G g_ji = G::Identity();
    for (int i = path_.size() - 1; i >= 0; --i) {
      // Get spanning tree edge and its transform
      const auto& [edge_kl, alongPath] = path_[i];
      const G g_kl = values.at<G>(edge_kl);

      // Check its orientation wrt path, pre-computed in constructor:
      if (alongPath) {
        // Edge (k,l) is oriented along path: invert g_kl to reverse-accumulate.
        if (H) H->at(i) = g_ji.AdjointMap();
        g_ji = g_ji * g_kl.inverse();
      } else {
        // Edge (k,l) is reversed in path: just use g_kl to reverse-accumulate.
        g_ji = g_ji * g_kl;
        // Non-obvious: putting this after accumulate avoids extra Ad and mult.
        if (H) H->at(i) = -g_ji.AdjointMap();
      }
    }

    // The predicted transform g_ij is the inverse of (now complete) g_ji.
    return g_ji.inverse();
  }

  /**
   * \brief Evaluate the error.
   */
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    // Predict the measurement, with optional derivatives.
    const G g_ij = predict(values, H);
    G between = measured_ji_ * g_ij;  // inv(\tilde g_ij) g_ij

    // Calculate error as log(inv(\tilde g_ij) g_ij)
    Matrix DLog;
    const Vector error = G::Logmap(between, H ? &DLog : nullptr);

    // differential of Logmap is typically approximated as identity, but if not:
    if (H && !DLog.isIdentity())
      for (auto H_i : *H) H_i = DLog * H_i;  // chain with DLog!

    return error;
  }

  /// Clone the factor.
  std::shared_ptr<NonlinearFactor> clone() const override {
    return std::make_shared<PathFactor<G>>(*this);
  }
};

template <class G>
const Matrix PathFactor<G>::Id_ = Matrix::Identity(G::dimension, G::dimension);

}  // namespace gtsam
