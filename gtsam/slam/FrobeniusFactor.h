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
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

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
 * @param n The desired dimension for the isotropic model.
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
 * Throws if the model's dimension does not match T's expected dimension
 * before conversion.
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

template <class T>
using FrobeniusErrorVector = Eigen::Matrix<
    double, T::LieAlgebra::RowsAtCompileTime *
                T::LieAlgebra::RowsAtCompileTime,
    1>;

/**
 * FrobeniusPrior calculates the Frobenius norm between a given matrix and a
 * fixed-size matrix Lie group element.
 */
template <class T>
class FrobeniusPrior : public NoiseModelFactorN<T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;

 public:
  using MatrixNN = Eigen::Matrix<double, N, N>;

 private:
  Eigen::Matrix<double, Dim, 1> vecM_;  ///< vectorized matrix to approximate

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor1<T>::evaluateError;

  /// Constructor
  FrobeniusPrior(Key j, const MatrixNN& M,
                 const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T>(ConvertModel<T, Dim>(model), j) {
    vecM_ << Eigen::Map<const Matrix>(M.data(), Dim, 1);
  }

  /// Return the fixed ambient matrix targeted by this prior.
  MatrixNN priorMatrix() const {
    return Eigen::Map<const MatrixNN>(vecM_.data());
  }

  /// Error is just Frobenius norm between T element and vectorized matrix M.
  Vector evaluateError(const T& g, OptionalMatrixType H) const override {
    return traits<T>::Vec(g, H) -
           vecM_;  // Jacobian is computed only when needed.
  }

  /**
   * Add this Frobenius prior to the QCQP graph.
   *
   * D=1 supports hard constrained Rot2, Rot3, Pose2, and Pose3 priors in their
   * exact homogeneous lifts. Matrix-form priors are not lowered because a
   * fixed lifted target breaks the right-orthogonal gauge required by the
   * Burer--Monteiro formulation.
   */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension == 0) {
      throw std::invalid_argument(
          "FrobeniusPrior::qcqpFactors: columnDimension must be >= 1");
    }
    if (columnDimension == 1) {
      qcqpFactorsForVec(costs, constraints);
    } else {
      qcqpFactorsForMatrix(costs, constraints, columnDimension);
    }
  }

 private:
  /// D=1 constrained-noise prior in lifted vector form.
  /// The stored measurement is vecM_ = vec(M), where M is the matrix passed to
  /// the FrobeniusPrior constructor. Pose lifts retain only the variable top
  /// rows of M; rotation lifts retain the full matrix.
  void qcqpFactorsForVec(NonlinearFactorGraph* costs,
                         NonlinearEqualityConstraints* constraints) const {
    if constexpr (!internal::HasQcqpVariableTraits<T, 1>::value) {
      (void)costs;
      (void)constraints;
      throw std::runtime_error(
          "FrobeniusPrior::qcqpFactors requires QCQP variable traits for this "
          "type and column dimension 1.");
    } else if constexpr (!(std::is_same_v<T, Rot2> || std::is_same_v<T, Rot3> ||
                           std::is_same_v<T, Pose2> ||
                           std::is_same_v<T, Pose3>)) {
      (void)costs;
      (void)constraints;
      throw std::runtime_error(
          "FrobeniusPrior::qcqpFactors D=1 is implemented only for Rot2, "
          "Rot3, Pose2, and Pose3.");
    } else {
      (void)costs;
      if (this->noiseModel_->isConstrained()) {
        InsertQcqpConstraints<T, 1>(this->key(), constraints);

        constexpr int LiftedDim = traits<T>::QcqpVectorDim;
        static_assert((LiftedDim - 1) % N == 0);
        constexpr int M = (LiftedDim - 1) / N;

        // Build the homogeneous target while omitting fixed pose-matrix rows.
        Vector target = Vector::Zero(LiftedDim);
        target(0) = 1.0;
        for (int column = 0; column < N; ++column) {
          target.segment<M>(1 + column * M) =
              vecM_.template segment<M>(column * N);
        }

        constraints->push_back(
            LinearConstraint::Equal(
                JacobianFactor(this->key(),
                               Matrix::Identity(LiftedDim, LiftedDim), target))
                .createEqualityFactor());
      } else {
        throw std::runtime_error(
            "FrobeniusPrior::qcqpFactors D=1 non-constrained noise is not yet "
            "implemented.");
      }
    }
  }

  /**
   * Reject matrix-form priors until a BM-compatible anchor-block lowering is
   * available. A future lowering can represent a prior as a gauge-invariant
   * between cost ||X-M'X_anchor||_F^2.
   */
  void qcqpFactorsForMatrix(NonlinearFactorGraph* costs,
                            NonlinearEqualityConstraints* constraints,
                            size_t columnDimension) const {
    (void)costs;
    (void)constraints;
    (void)columnDimension;
    throw std::runtime_error(
        "FrobeniusPrior::qcqpFactors does not support matrix-form priors; "
        "a Burer--Monteiro-compatible prior requires an anchor block.");
  }
};

/**
 * FrobeniusFactor calculates the Frobenius norm between matrix Lie group
 * elements.
 */
template <class T>
class FrobeniusFactor
    : public NoiseModelFactorT<FrobeniusErrorVector<T>, T, T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;
  using Base = NoiseModelFactorT<FrobeniusErrorVector<T>, T, T>;
  using VectorD = FrobeniusErrorVector<T>;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// Constructor
  FrobeniusFactor(Key j1, Key j2, const SharedNoiseModel& model = nullptr)
      : Base(ConvertModel<T, Dim>(model), j1, j2) {}

  /// Error is the vectorized matrix difference between the two group elements.
  VectorD evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                        OptionalMatrixType H2) const override {
    VectorD error = traits<T>::Vec(T2, H2) - traits<T>::Vec(T1, H1);
    if (H1) *H1 = -*H1;
    return error;
  }
};

/**
 * FrobeniusBetweenFactorNL is a BetweenFactor that evaluates the Frobenius
 * norm of the matrix error between measured and predicted (rather than the
 * Logmap of the error). This factor is only defined for fixed-dimension
 * matrix Lie groups.
 *
 * This version is called NL, because it minimizes |inv(T2)*T1*T12_ - I|_F
 * as opposed to the (historically older) FrobeniusBetweenFactor, that
 * minimizes
 * ||T2 - T1*T12_||_F. This only holds for certain groups, e.g., not Sim(3).
 */
template <class T>
class FrobeniusBetweenFactorNL
    : public NoiseModelFactorT<FrobeniusErrorVector<T>, T, T> {
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;
  static_assert(N > 0, "The Lie algebra dimension N must be greater than 0.");

 protected:
  T T12_;  ///< measured transformation between T1 and T2

  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorD = Eigen::Matrix<double, Dim, 1>;
  using Base = NoiseModelFactorT<VectorD, T, T>;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// @name Constructor
  /// @{

  /// Construct from two keys and a measured transformation.
  FrobeniusBetweenFactorNL(Key j1, Key j2, const T& T12,
                           const SharedNoiseModel& model = nullptr)
      : Base(ConvertModel<T, Dim>(model), j1, j2),
        T12_(T12) {}

  /// Return the measured transformation from the first key to the second.
  const T& measured() const { return T12_; }

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
    const auto* e = dynamic_cast<const FrobeniusBetweenFactorNL*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<T>::Equals(this->T12_, e->T12_, tol);
  }

  /// @}
  /// @name NoiseModelFactorN methods
  /// @{

  /// Error is |inv(T2)*T1*T12_ - I|_F.
  VectorD evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                        OptionalMatrixType H2) const override {
    const bool computeJacobians = H1 || H2;

    // Compute the predicted inverse-relative transform inv(T2) * T1.
    typename T::Jacobian H_T21_T2;
    const T hatT21 =
        traits<T>::Between(T2, T1, computeJacobians ? &H_T21_T2 : nullptr);

    // Compose with the measurement; the result should be identity.
    typename T::Jacobian H_pred_hat = T::Jacobian::Zero();
    const T pred = traits<T>::Compose(hatT21, T12_,
                                      computeJacobians ? &H_pred_hat : nullptr);

    // Cache the fixed-size vectorization of the identity matrix.
    static const VectorD vecI = [] {
      const MatrixN I = MatrixN::Identity();
      return VectorD(Eigen::Map<const VectorD>(I.data()));
    }();

    // Vectorize the residual and retain its derivative for the chain rule.
    Eigen::Matrix<double, Dim, T::dimension> H_vec_pred;
    VectorD error =
        traits<T>::Vec(pred, computeJacobians ? &H_vec_pred : nullptr) - vecI;

    // Propagate derivatives through Between and Compose.
    if (computeJacobians) {
      const auto H_error_hat21 = H_vec_pred * H_pred_hat;
      if (H1) *H1 = H_error_hat21;  // H_pred_T1 is identity
      if (H2) *H2 = H_error_hat21 * H_T21_T2;
    }
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
  using Base = FrobeniusBetweenFactorNL<T>;
  using VectorD = FrobeniusErrorVector<T>;

  typename T::Jacobian T2hat_H_T1_;  ///< fixed derivative of T2hat wrpt T1

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// Construct from two keys and a measured transformation.
  FrobeniusBetweenFactor(Key j1, Key j2, const T& T12,
                         const SharedNoiseModel& model = nullptr)
      : FrobeniusBetweenFactorNL<T>(j1, j2, T12, model),
        T2hat_H_T1_(traits<T>::AdjointMap(traits<T>::Inverse(T12))) {}

  /// Error is Frobenius norm between T1*T12 and T2.
  VectorD evaluateError(const T& T1, const T& T2, OptionalMatrixType H1,
                        OptionalMatrixType H2) const override {
    const T T2hat = traits<T>::Compose(T1, this->T12_);
    Eigen::Matrix<double, Dim, T::dimension> vec_H_T2hat;
    VectorD error = traits<T>::Vec(T2, H2) -
                    traits<T>::Vec(T2hat, H1 ? &vec_H_T2hat : nullptr);
    if (H1) *H1 = -vec_H_T2hat * T2hat_H_T1_;
    return error;
  }

  /**
   * Add this Frobenius between factor as a QCQP cost when traits exist.
   *
   * D=1 takes the exact homogeneous Rot2, Rot3, Pose2, or Pose3 form. For Rot2
   * and Rot3, D>=N takes the N-by-D row-Stiefel form, where N is the intrinsic
   * rotation-matrix dimension.
   */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension == 0) {
      throw std::invalid_argument(
          "FrobeniusBetweenFactor::qcqpFactors: columnDimension must be >= 1");
    }
    if (columnDimension == 1) {
      qcqpFactorsForVec(costs, constraints);
    } else {
      qcqpFactorsForMatrix(costs, constraints, columnDimension);
    }
  }

 private:
  /// Compute a Kronecker product without depending on unsupported Eigen
  /// modules.
  static Matrix internalKroneckerProduct(const Matrix& left,
                                         const Matrix& right) {
    const DenseIndex resultRows = left.rows() * right.rows();
    const DenseIndex resultColumns = left.cols() * right.cols();
    Matrix result = Matrix::Zero(resultRows, resultColumns);

    for (DenseIndex row = 0; row < left.rows(); ++row) {
      for (DenseIndex column = 0; column < left.cols(); ++column) {
        result.block(row * right.rows(), column * right.cols(), right.rows(),
                     right.cols()) = left(row, column) * right;
      }
    }
    return result;
  }

  /// D=1 retained-row vector form: build the full Frobenius residual for
  /// whitening, then embed its quadratic matrix in the lifted coordinates.
  void qcqpFactorsForVec(NonlinearFactorGraph* costs,
                         NonlinearEqualityConstraints* constraints) const {
    if constexpr (!internal::HasQcqpVariableTraits<T, 1>::value) {
      (void)costs;
      (void)constraints;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors requires QCQP variable traits "
          "for this type and column dimension 1.");
    } else if constexpr (!(std::is_same_v<T, Rot2> || std::is_same_v<T, Rot3> ||
                           std::is_same_v<T, Pose2> ||
                           std::is_same_v<T, Pose3>)) {
      (void)costs;
      (void)constraints;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors D=1 is implemented only for "
          "Rot2, Rot3, Pose2, and Pose3.");
    } else {
      if (!costs) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactors costs is null");
      }
      if (std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
          this->noiseModel_->isConstrained()) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactors requires a "
            "non-robust/non-hard quadratic noise model");
      }

      constexpr int LiftedDim = traits<T>::QcqpVectorDim;
      constexpr int MN = LiftedDim - 1;  // The lifted vector has a leading 1
      static_assert(MN % N == 0);
      constexpr int M = MN / N;

      const Matrix measurement = this->T12_.matrix();
      const Matrix I_M = Matrix::Identity(M, M);
      // Column-major vec(XM) = (M.transpose() kron I) vec(X).
      const Matrix A = internalKroneckerProduct(measurement.transpose(), I_M);

      // Build the retained-coordinate residual B = [-A I].
      Matrix B = Matrix::Zero(MN, 2 * MN);
      B.block<MN, MN>(0, 0) = -A;
      B.block<MN, MN>(0, MN).setIdentity();

      // Restore omitted pose-matrix rows before applying the noise model.
      Matrix fullB = Matrix::Zero(Dim, 2 * MN);
      for (int column = 0; column < N; ++column) {
        fullB.block<M, 2 * MN>(column * N, 0) =
            B.block<M, 2 * MN>(column * M, 0);
      }

      const Matrix whitenedB = this->noiseModel_->Whiten(fullB);
      const Matrix Q = whitenedB.transpose() * whitenedB;

      // Embed both Hessian blocks into homogeneous QCQP coordinates.
      Matrix Q_trunc_hom = Matrix::Zero(2 * LiftedDim, 2 * LiftedDim);
      Q_trunc_hom.block<MN, MN>(1, 1) = Q.block<MN, MN>(0, 0);
      Q_trunc_hom.block<MN, MN>(1, LiftedDim + 1) = Q.block<MN, MN>(0, MN);
      Q_trunc_hom.block<MN, MN>(LiftedDim + 1, 1) = Q.block<MN, MN>(MN, 0);
      Q_trunc_hom.block<MN, MN>(LiftedDim + 1, LiftedDim + 1) =
          Q.block<MN, MN>(MN, MN);

      InsertQcqpConstraints<T, 1>(this->key1(), constraints);
      InsertQcqpConstraints<T, 1>(this->key2(), constraints);

      const SymmetricBlockMatrix blockQ(
          std::vector<DenseIndex>{LiftedDim, LiftedDim}, Q_trunc_hom);
      costs->push_back(std::make_shared<QpCost>(
          KeyVector{this->key1(), this->key2()}, blockQ));
    }
  }

  /// Matrix form (D>=N): N-by-D row-Stiefel variables and isotropic noise.
  void qcqpFactorsForMatrix(NonlinearFactorGraph* costs,
                            NonlinearEqualityConstraints* constraints,
                            size_t columnDimension) const {
    if constexpr (!internal::HasQcqpVariableTraits<T, N>::value) {
      (void)costs;
      (void)constraints;
      (void)columnDimension;
      throw std::runtime_error(
          "FrobeniusBetweenFactor::qcqpFactors requires QCQP variable traits "
          "for this type and matrix-form column dimensions (>= 2).");
    } else {
      if (columnDimension < static_cast<size_t>(N)) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactors: columnDimension must be "
            ">= the variable's intrinsic row dimension (e.g. >= 3 for Rot3).");
      }
      if (!costs) {
        throw std::invalid_argument(
            "FrobeniusBetweenFactor::qcqpFactors costs is null");
      }
      if (std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
          this->noiseModel_->isConstrained()) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactors requires a "
            "non-robust quadratic noise model");
      }
      const auto isotropic =
          std::dynamic_pointer_cast<noiseModel::Isotropic>(this->noiseModel_);
      if (!isotropic) {
        throw std::runtime_error(
            "FrobeniusBetweenFactor::qcqpFactors with column dimension > 1 "
            "requires an isotropic noise model");
      }

      InsertQcqpConstraints<T, N>(this->key1(), constraints);
      InsertQcqpConstraints<T, N>(this->key2(), constraints);

      const Matrix measurement = this->T12_.matrix();
      const Matrix I = Matrix::Identity(N, N);
      const double weight = 1.0 / (isotropic->sigma() * isotropic->sigma());

      // Build B = [-M' I] and its isotropically weighted Hessian.
      Matrix B = Matrix::Zero(N, 2 * N);
      B.block<N, N>(0, 0) = -measurement.transpose();
      B.block<N, N>(0, N) = I;

      const Matrix Q = weight * B.transpose() * B;
      const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{N, N}, Q);
      costs->push_back(std::make_shared<QpCost>(
          KeyVector{this->key1(), this->key2()}, blockQ, columnDimension));
    }
  }
};

}  // namespace gtsam
