//------------------------------------------------------------------------------
// PathFactor.h
//------------------------------------------------------------------------------
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/types.h>
#include <vector>
#include <stdexcept>
#include <gtsam/inference/EdgeKey.h>


namespace gtsam {

/**
 * \brief PathFactor
 *
 * A NonlinearFactor defined on a sequence (path) of relative rotations (or, more generally,
 * Lie group elements) whose predicted overall transformation is given by the composition
 * along the path.
 *
 * Template parameter G is assumed to be a LieGroup derivative.
 */
template<class G>
class PathFactor : public NonlinearFactor {
public:
  typedef std::shared_ptr<PathFactor> shared_ptr;

private:
  /// Ordered sequence of EdgeKeys forming the path.
  std::vector<EdgeKey> pathKeys_;
  /// Measured overall relative transformation along the path.
  G measured_;

  /// Helper: Extract Keys from a vector of EdgeKey.
  static KeyVector keysFromEdgeKeys(const std::vector<EdgeKey>& keys) {
    KeyVector keyVec;
    for (const auto& ek : keys) {
      keyVec.push_back(static_cast<Key>(ek));
    }
    return keyVec;
  }

  /**
   * \brief Compute effective measurements along the path.
   *
   * For each EdgeKey in the path, check whether the Values object contains the
   * measurement stored as \(\mathtt{g}_{ij}\) (forward) or as \(\mathtt{g}_{ji}\)
   * (reversed). In the latter case, the effective measurement is taken as
   * \(\mathtt{g}_{ji}^{-1}\) and the local derivative is \(-\operatorname{Ad}_{\mathtt{g}_{ji}}\).
   *
   * \param c The Values object.
   * \param Qs (Output) Vector of effective measurements.
   * \param localDerivatives (Output) Local derivative for each measurement.
   * \param prediction (Output) The overall composed product along the path.
   */
  void computeEffectivePath(const Values& c,
                            std::vector<G>& Qs,
                            std::vector<Matrix>& localDerivatives,
                            G& prediction) const {
    const size_t d = G::dimension;
    Qs.clear();
    localDerivatives.clear();
    prediction = G::Identity();
    for (const auto& ek : pathKeys_) {
      Key k = static_cast<Key>(ek);
      bool forward = c.exists(k);
      if (forward) {
        G gij = c.at<G>(k);
        Qs.push_back(gij);
        localDerivatives.push_back(Matrix::Identity(d, d));
      } else {
        Key k_rev = static_cast<Key>(ek.reversed());
        G gji = c.at<G>(k_rev);
        Qs.push_back(gji.inverse());
        localDerivatives.push_back(-gji.AdjointMap());
      }
      prediction = prediction.compose(Qs.back());
    }
  }

public:
  /**
   * \brief Constructor.
   * \param pathKeys Vector of EdgeKey representing the unique path.
   * \param measured The measured overall transformation (e.g., from a loop closure).
   */
  PathFactor(const std::vector<EdgeKey>& pathKeys, const G& measured)
      : NonlinearFactor(keysFromEdgeKeys(pathKeys)),
        pathKeys_(pathKeys), measured_(measured) {}

  /// Return the factor dimension (Lie algebra dimension of G).
  virtual size_t dim() const override {
    return G::dimension;
  }

  /**
   * \brief Evaluate the error.
   *
   * For a given set of values \c c, compute the predicted overall transformation by composing
   * the measurements along the path. For each EdgeKey, the measurement may be stored either as
   * \(\mathtt{g}_{ij}\) or \(\mathtt{g}_{ji}\). If \(\mathtt{g}_{ij}\) is present, it is used directly;
   * otherwise, the measurement \(\mathtt{g}_{ji}\) is used and inverted.
   *
   * The error is defined as the squared norm of the Logmap of
   * \(\text{measured}^{-1} \circ \text{prediction}\).
   */
  virtual double error(const Values& c) const override {
    G prediction;
    std::vector<G> Qs;
    std::vector<Matrix> localDerivatives;
    computeEffectivePath(c, Qs, localDerivatives, prediction);
    G residual = measured_.inverse().compose(prediction);
    return G::Logmap(residual).squaredNorm();
  }

  /**
   * \brief Linearize the factor.
   *
   * This method computes the Jacobian by applying the chain rule along the path.
   * For a product \( h = Q_1 \cdots Q_n \) (with \( Q_k \) taken from the Values),
   * we backpropagate the adjoint maps:
   * \[
   * A_k = \prod_{l=k+1}^{n} \operatorname{Ad}_{Q_l^{-1}},
   * \]
   * and obtain the Jacobian with respect to the \(k\)th measurement as
   * \[
   * J_k = D\Log(H) \, A_k \, L_k,
   * \]
   * where \( H = \text{measured}^{-1} \circ \text{prediction} \) and \( L_k \) is the local
   * derivative (\(I\) when stored forward and \(-\operatorname{Ad}\) when reversed).
   */
  virtual std::shared_ptr<GaussianFactor> linearize(const Values& c) const override {
    const size_t d = G::dimension;
    std::vector<G> Qs;
    std::vector<Matrix> localDerivatives;
    G prediction;
    computeEffectivePath(c, Qs, localDerivatives, prediction);

    // Compute residual: H = measured^{-1} * prediction.
    G H = measured_.inverse().compose(prediction);
    Matrix DLog;
    Vector b = G::Logmap(H, DLog);

    // Backpropagate adjoint maps:
    // A[k] = \prod_{l=k+1}^{n} Ad_{Q_l^{-1}}, with A[n] = I.
    std::vector<Matrix> A(pathKeys_.size(), Matrix::Identity(d, d));
    Matrix accum = Matrix::Identity(d, d);
    for (int k = static_cast<int>(pathKeys_.size()) - 1; k >= 0; --k) {
      A[k] = accum;
      accum = Qs[k].inverse().AdjointMap() * accum;
    }

    // Assemble the Jacobians.
    std::vector<std::pair<Key, Matrix>> jacobians;
    for (size_t k = 0; k < pathKeys_.size(); ++k) {
      Matrix J = DLog * A[k] * localDerivatives[k];
      jacobians.push_back(std::make_pair(static_cast<Key>(pathKeys_[k]), J));
    }
    return std::make_shared<JacobianFactor>(jacobians, b);
  }

  /// Clone the factor.
  virtual std::shared_ptr<NonlinearFactor> clone() const override{
    return std::make_shared<PathFactor<G>>(*this);
  }
};

} // namespace gtsam
