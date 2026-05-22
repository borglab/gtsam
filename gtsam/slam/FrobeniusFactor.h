/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FrobeniusFactor.h
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Various factors that minimize some Frobenius norm
 */

#pragma once

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <stdexcept>

namespace gtsam {

/**
 * @brief Convert a possibly robust noise model to an isotropic model for
 * Frobenius factors.
 *
 * This function is used to convert a noise model, which may be robust, into an
 * isotropic noise model suitable for Frobenius factors. If the input noise
 * model is null, it returns an n-dimensional isotropic noise model with
 * sigma=1.0. If the input noise model is isotropic, it extends it to the
 * desired dimension. If the noise model is robust, the sigmas of the underlying
 * noise model are used. If the noise model is not isotropic and `defaultToUnit`
 * is false, an exception is thrown.
 *
 * @param model The input noise model (possibly robust).
 * @param dimension The desired dimension for the isotropic model.
 * @param defaultToUnit If true, fallback to unit if conversion is not possible.
 * @throws std::runtime_error if model not isotropic and defaultToUnit =false.
 * @return An isotropic (possibly robust) noise model.
 */
GTSAM_EXPORT SharedNoiseModel ConvertNoiseModel(const SharedNoiseModel& model,
                                                size_t n,
                                                bool defaultToUnit = true);

/**
 * @brief Ensure a noise model has the correct dimension for a given type.
 *
 * If the model is already of dimension Dim, it is returned as-is.
 * Otherwise, ConvertNoiseModel is called to convert it.
 * Asserts that the model's dimension matches T's expected dimension before
 * conversion.
 *
 * @tparam T The type whose dimension is checked.
 * @tparam Dim The required dimension.
 * @param model The input noise model.
 * @return A noise model of dimension Dim.
 */
template <class T, size_t Dim>
inline SharedNoiseModel ConvertModel(const SharedNoiseModel& model) {
  if (!model) {
    return ConvertNoiseModel(noiseModel::Unit::Create(T()), Dim);
  }
  if (model->dim() == Dim) {
    return model;
  }
  if (model->dim() != T::dimension) {
    throw std::runtime_error(
        "Noise model dimension does not match expected dimension for T.");
  }
  return ConvertNoiseModel(model, Dim);
}

/**
 * FrobeniusPrior calculates the Frobenius norm between a given matrix and an
 * element of SO(3) or SO(4).
 */
template <class T>
class FrobeniusPrior : public NoiseModelFactorN<T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;
  using MatrixNN = Eigen::Matrix<double, N, N>;
  Eigen::Matrix<double, Dim, 1> vecM_;  ///< vectorized matrix to approximate

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor1<T>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor
  FrobeniusPrior(Key j, const MatrixNN& M,
                 const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T>(ConvertModel<T, Dim>(model), j) {
    vecM_ << Eigen::Map<const Matrix>(M.data(), Dim, 1);
  }

  /// Error is just Frobenius norm between T element and vectorized matrix M.
  Vector evaluateError(const T& g, OptionalMatrixType H) const override {
    return traits<T>::Vec(g, H) -
           vecM_;  // Jacobian is computed only when needed.
  }
};

/**
 * FrobeniusFactor calculates the Frobenius norm between rotation matrices.
 * The template argument can be any fixed-size SO<N>.
 */
template <class T>
class FrobeniusFactor : public NoiseModelFactorN<T, T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<T, T>::evaluateError;

  /// Constructor
  FrobeniusFactor(Key j1, Key j2, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T, T>(ConvertModel<T, Dim>(model), j1, j2) {}

  /// Error is just Frobenius norm between rotation matrices.
  Vector evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    Vector error = traits<T>::Vec(T2, H2) - traits<T>::Vec(T1, H1);
    if (H1) *H1 = -*H1;
    return error;
  }
};

/**
 * FrobeniusBetweenFactorNL is a BetweenFactor that evaluates the Frobenius
 * norm of the rotation error between measured and predicted (rather than the
 * Logmap of the error). This factor is only defined for fixed-dimension
 * types, that are matrix Lie groups.
 *
 * This version is called NL, because it minimizes |inv(T2)*T1*T12_ - I|_F
 * as opposed to the (historically older) FrobeniusBetweenFactor, that
 * minimizes
 * ||T2 - T1*T12_||_F. This only holds for certain groups, e.g., not Sim(3).
 */
template <class T>
class FrobeniusBetweenFactorNL : public NoiseModelFactorN<T, T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;
  static_assert(N > 0, "The Lie algebra dimension N must be greater than 0.");

 protected:
  T T12_;  ///< measured rotation between T1 and T2

  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorD = Eigen::Matrix<double, Dim, 1>;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<T, T>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructor
  /// @{

  /// Construct from two keys and measured rotation
  FrobeniusBetweenFactorNL(Key j1, Key j2, const T& T12,
                           const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T, T>(ConvertModel<T, Dim>(model), j1, j2),
        T12_(T12) {}

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "FrobeniusBetweenFactorNL<" << demangle(typeid(T).name())
              << ">(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    traits<T>::Print(T12_, "  T12: ");
    this->noiseModel_->print("  noise model: ");
  }

  /// assert equality up to a tolerance
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    auto e = dynamic_cast<const FrobeniusBetweenFactorNL*>(&expected);
    return e != nullptr && NoiseModelFactorN<T, T>::equals(*e, tol) &&
           traits<T>::Equals(this->T12_, e->T12_, tol);
  }

  /// @}
  /// @name NoiseModelFactorN methods
  /// @{

  /// Error is |inv(T2)*T1*T12_ - I|_F.
  Vector evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    // predict T2*T1
    typename T::Jacobian H_T21_T2;
    const T hatT21 = traits<T>::Between(T2, T1, H1 ? &H_T21_T2 : nullptr);

    // Calculate \hat T21 * T12_, which is predicted to be I_NxN
    typename T::Jacobian H_pred_hat = T::Jacobian::Zero();
    const T pred = traits<T>::Compose(hatT21, T12_, H1 ? &H_pred_hat : nullptr);

    // Move to constructor
    const MatrixN I = MatrixN::Identity();
    const VectorD vecI = Eigen::Map<const VectorD>(I.data());

    // Calculate error
    Eigen::Matrix<double, Dim, T::dimension> H_vec_pred;
    Vector error = traits<T>::Vec(pred, H1 ? &H_vec_pred : nullptr) - vecI;

    // Do chain rule
    const auto H_error_hat21 = H_vec_pred * H_pred_hat;
    if (H1) *H1 = H_error_hat21;  // H_pred_T1 is identity
    if (H2) *H2 = H_error_hat21 * H_T21_T2;
    return error;
  }
  /// @}
};

/**
 * FrobeniusBetweenFactor uses ||T2 - T1*T12_||_F, which only works if the
 * Frobenius error is invariant to multiplying with an arbitrary element T.
 */
template <class T>
class FrobeniusBetweenFactor : public FrobeniusBetweenFactorNL<T> {
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;

  typename T::Jacobian T2hat_H_T1_;  ///< fixed derivative of T2hat wrpt T1

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<T, T>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Construct from two keys and measured rotation
  FrobeniusBetweenFactor(Key j1, Key j2, const T& T12,
                         const SharedNoiseModel& model = nullptr)
      : FrobeniusBetweenFactorNL<T>(j1, j2, T12, model),
        T2hat_H_T1_(traits<T>::AdjointMap(traits<T>::Inverse(T12))) {}

  /// Error is Frobenius norm between T1*T12 and T2.
  Vector evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    const T T2hat = traits<T>::Compose(T1, this->T12_);
    Eigen::Matrix<double, Dim, T::dimension> vec_H_T2hat;
    Vector error = traits<T>::Vec(T2, H2) -
                   traits<T>::Vec(T2hat, H1 ? &vec_H_T2hat : nullptr);
    if (H1) *H1 = -vec_H_T2hat * T2hat_H_T1_;
    return error;
  }

  /**
   * Emit QCQP cost + variable constraints for this edge at column dim K.
   * `QcqpProblem::rebuildAt(K)` calls this to re-emit the edge at any K:
   *   K = 1     vec(R) form (polynomial relaxation).
   *   K = d     natural matrix form of the QCQP.
   *   K > d     low-rank factorization used by the Riemannian Staircase.
   * Q is doubled in the helpers so GTSAM's 0.5 cost convention cancels:
   * `qcqp.costs().error(X) == 2 * graph.error(X)`.
   */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t K) const override {
    if (K == 0) {
      throw std::invalid_argument(
          "FrobeniusBetweenFactor::qcqpFactors: K must be >= 1.");
    }
    if (K == 1) {
      qcqpFactorsForVec(costs, constraints);
    } else {
      qcqpFactorsForMatrix(costs, constraints, K);
    }
  }

 private:
  /// vec(X R) = RightProductMatrix(R) * vec(X) for N×N matrices X, R.
  static Matrix RightProductMatrix(const Matrix& right) {
    constexpr int AmbientDim = N * N;
    const Matrix I = Matrix::Identity(N, N);
    Matrix result = Matrix::Zero(AmbientDim, AmbientDim);
    for (int column = 0; column < N; ++column) {
      for (int sourceColumn = 0; sourceColumn < N; ++sourceColumn) {
        result.block(column * N, sourceColumn * N, N, N) =
            right(sourceColumn, column) * I;
      }
    }
    return result;
  }

  /// Vec(R) form (K = 1). Variable is (N*N)x1; vec(X*R) is linear in
  /// vec(X) via RightProductMatrix(R). The `if constexpr` keeps this
  /// compilable for T = SO<n>, which has no QCQP traits; the runtime
  /// throw fires if it is actually invoked.
  void qcqpFactorsForVec(NonlinearFactorGraph* costs,
                         NonlinearEqualityConstraints* constraints) const {
    if constexpr (!internal::HasQcqpVariableTraits<T, 1>::value) {
      (void)costs;
      (void)constraints;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors requires QCQP variable traits "
          "(traits<T>::QcqpConstraints<1>) for this variable type.");
    } else {
      if (!costs) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactors: costs is null");
      }
      if (std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
          this->noiseModel_->isConstrained()) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactors requires a non-robust "
            "quadratic noise model.");
      }

      InsertQcqpConstraints<T, 1>(this->key1(), constraints);
      InsertQcqpConstraints<T, 1>(this->key2(), constraints);

      constexpr int AmbientDim = N * N;
      const Matrix measurement = this->T12_.matrix();
      const Matrix A = RightProductMatrix(measurement);
      Matrix B = Matrix::Zero(AmbientDim, 2 * AmbientDim);
      B.block(0, 0, AmbientDim, AmbientDim) = -A;
      B.block(0, AmbientDim, AmbientDim, AmbientDim) =
          Matrix::Identity(AmbientDim, AmbientDim);
      const Matrix whitenedB = this->noiseModel_->Whiten(B);
      // 2× absorbs QpCost's 0.5 convention; paper cost is ||residual||^2.
      const Matrix Q = 2.0 * whitenedB.transpose() * whitenedB;
      const SymmetricBlockMatrix blockQ(
          std::vector<DenseIndex>{AmbientDim, AmbientDim}, Q);
      costs->push_back(std::make_shared<QpCost>(
          KeyVector{this->key1(), this->key2()}, blockQ, /*columnDim=*/1));
    }
  }

  /// Matrix form (K >= 2). Builds the d-by-d Frobenius Hessian Q_ij and
  /// emits the trait's row-orthonormality constraints A_m. Both are
  /// K-invariant; QpCost Kronecker-expands them with `columnDim = K`, so
  /// climbing K -> K+1 reuses the same Q_ij and A_m and only changes the
  /// Kronecker dim.
  void qcqpFactorsForMatrix(NonlinearFactorGraph* costs,
                            NonlinearEqualityConstraints* constraints,
                            size_t K) const {
    if constexpr (!internal::HasQcqpIntrinsicDim<T>::value) {
      (void)costs;
      (void)constraints;
      (void)K;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors requires QCQP variable traits "
          "(traits<T>::QcqpIntrinsicDim) for this variable type.");
    } else if constexpr (!internal::HasQcqpVariableTraits<
                             T, traits<T>::QcqpIntrinsicDim>::value) {
      (void)costs;
      (void)constraints;
      (void)K;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors requires QCQP variable traits "
          "(traits<T>::QcqpConstraints<QcqpIntrinsicDim>) for this variable "
          "type.");
    } else {
      constexpr int RepD = traits<T>::QcqpIntrinsicDim;
      if (K < static_cast<size_t>(RepD)) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactorsForMatrix: K must be >= "
            "traits<T>::QcqpIntrinsicDim (e.g. >= 3 for Rot3).");
      }
      if (!costs) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactorsForMatrix: costs is null");
      }
      if (std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
          this->noiseModel_->isConstrained()) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactorsForMatrix requires a "
            "non-robust quadratic noise model.");
      }
      const auto isotropic =
          std::dynamic_pointer_cast<noiseModel::Isotropic>(this->noiseModel_);
      if (!isotropic) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactorsForMatrix requires an "
            "isotropic noise model in matrix form (K >= 2).");
      }

      InsertQcqpConstraints<T, RepD>(this->key1(), constraints);
      InsertQcqpConstraints<T, RepD>(this->key2(), constraints);

      const Matrix measurement = this->T12_.matrix();
      const Matrix I = Matrix::Identity(N, N);
      const double weight = 1.0 / (isotropic->sigma() * isotropic->sigma());

      Matrix B = Matrix::Zero(N, 2 * N);
      B.block(0, 0, N, N) = -measurement.transpose();
      B.block(0, N, N, N) = I;

      // 2× absorbs QpCost's 0.5 convention; paper cost is ||residual||^2.
      const Matrix Q = 2.0 * weight * B.transpose() * B;
      const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{N, N}, Q);
      costs->push_back(std::make_shared<QpCost>(
          KeyVector{this->key1(), this->key2()}, blockQ, /*columnDim=*/K));
    }
  }
};

}  // namespace gtsam
