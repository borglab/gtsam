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
    for (const auto& edge : path) {
      if (edge.i() != current) {
        throw std::invalid_argument(
            "Path is not continuous at edge starting with " +
            std::to_string(edge.i()));
      }
      current = edge.j();
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
   * \brief Compute effective measurements along the path.
   *
   * For each EdgeKey in the path, check whether the Values object contains the
   * measurement stored as \(\mathtt{g}_{ij}\) (forward) or as
   * \(\mathtt{g}_{ji}\) (reversed). In the latter case, the effective
   * measurement is taken as
   * \(\mathtt{g}_{ji}^{-1}\) and the local derivative is
   * \(-\operatorname{Ad}_{\mathtt{g}_{ji}}\).
   *
   * \param c The Values object.
   * \param Qs (Output) Vector of effective measurements.
   * \param localDerivatives (Output) Local derivative for each measurement.
   * \param prediction (Output) The overall composed product along the path.
   */
  G computeEffectivePath(
      const Values& values,
      std::vector<std::pair<G, Matrix>>* Qs = nullptr) const {
    G prediction = G::Identity();
    for (const auto& ek : path_) {
      G Q;
      bool forward = values.exists(ek);
      if (forward) {
        Q = values.at<G>(ek);  // gij
        if (Qs) Qs->emplace_back(Q, Id_);
      } else {
        G gji = values.at<G>(ek.reversed());
        Q = gji.inverse();  // inv(gij)
        if (Qs) Qs->emplace_back(Q, -gji.AdjointMap());
      }
      prediction = prediction * Q;
    }
    return prediction;
  }

  /**
   * \brief Evaluate the error.
   *
   * For a given set of values \c c, compute the predicted overall
   * transformation by composing the measurements along the path. For each
   * EdgeKey, the measurement may be stored either as
   * \(\mathtt{g}_{ij}\) or \(\mathtt{g}_{ji}\). If \(\mathtt{g}_{ij}\) is
   * present, it is used directly; otherwise, the measurement
   * \(\mathtt{g}_{ji}\) is used and inverted.
   *
   * The error is defined as the squared norm of the Logmap of
   * \(\text{measured}^{-1} \circ \text{prediction}\).
   */
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    std::vector<std::pair<G, Matrix>> Qs;
    const G prediction = computeEffectivePath(values, H ? &Qs : nullptr);
    G residual = measured_.between(prediction);

    if (H) {
      Matrix DLog;
      Vector b = G::Logmap(residual, DLog);

      // Backpropagate adjoint maps:
      // A[k] = \prod_{l=k+1}^{n} Ad_{Q_l^{-1}}, with A[n] = I.
      std::vector<Matrix> A(path_.size(), Id_);
      Matrix accum = Id_;
      for (int k = static_cast<int>(path_.size()) - 1; k >= 0; --k) {
        A[k] = accum;
        auto [Q, _] = Qs[k];
        accum = Q.inverse().AdjointMap() * accum;
      }

      // Assemble the Jacobians.
      for (size_t k = 0; k < path_.size(); ++k) {
        auto [_, localDerivative] = Qs[k];
        H->at(k) = DLog * A[k] * localDerivative;
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
