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

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * When creating (any) FrobeniusFactor we can convert a Rot/Pose BetweenFactor
 * noise model into a n-dimensional isotropic noise
 * model used to weight the Frobenius norm.
 * If the noise model passed is null we return a n-dimensional isotropic noise
 * model with sigma=1.0.
 * If not, we we check if the d-dimensional noise model on rotations is
 * isotropic. If it is, we extend to 'n' dimensions, otherwise we throw an
 * error. If the noise model is a robust error model, we use the sigmas of the
 * underlying noise model.
 *
 * If defaultToUnit == false throws an exception on unexpected input.
 */
GTSAM_EXPORT SharedNoiseModel
ConvertNoiseModel(const SharedNoiseModel &model, size_t n,
                  bool defaultToUnit = true);

/**
 * FrobeniusPrior calculates the Frobenius norm between a given matrix and an
 * element of SO(3) or SO(4).
 */
template <class T>
class FrobeniusPrior : public NoiseModelFactorN<T> {
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
      : NoiseModelFactorN<T>(ConvertNoiseModel(model, Dim), j) {
    vecM_ << Eigen::Map<const Matrix>(M.data(), Dim, 1);
  }

  /// Error is just Frobenius norm between T element and vectorized matrix M.
  Vector evaluateError(const T& g, OptionalMatrixType H) const override {
    return g.vec(H) - vecM_;  // Jacobian is computed only when needed.
  }
};

/**
 * FrobeniusFactor calculates the Frobenius norm between rotation matrices.
 * The template argument can be any fixed-size SO<N>.
 */
template <class T>
class FrobeniusFactor : public NoiseModelFactorN<T, T> {
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;

 public:

  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<T, T>::evaluateError;

  /// Constructor
  FrobeniusFactor(Key j1, Key j2, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T, T>(ConvertNoiseModel(model, Dim), j1, j2) {}

  /// Error is just Frobenius norm between rotation matrices.
  Vector evaluateError(const T& T1, const T& T2,
                       OptionalMatrixType H1, OptionalMatrixType H2) const override {
    Vector error = T2.vec(H2) - T1.vec(H1);
    if (H1) *H1 = -*H1;
    return error;
  }
};

/**
 * FrobeniusBetweenFactor is a BetweenFactor that evaluates the Frobenius norm
 * of the rotation error between measured and predicted (rather than the
 * Logmap of the error). This factor is only defined for fixed-dimension types,
 * and in fact only SO3 and SO4 really work, as we need SO<N>::AdjointMap.
 */
template <class T>
class FrobeniusBetweenFactor : public NoiseModelFactorN<T, T> {
  T T12_;  ///< measured rotation between T1 and T2
  Eigen::Matrix<double, T::dimension, T::dimension>
      T2hat_H_T1_;  ///< fixed derivative of T2hat wrpt T1
  inline constexpr static auto N = T::LieAlgebra::RowsAtCompileTime;
  inline constexpr static auto Dim = N * N;
    
 public:

  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<T, T>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructor
  /// @{

  /// Construct from two keys and measured rotation
  FrobeniusBetweenFactor(Key j1, Key j2, const T& T12,
                         const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<T, T>(ConvertNoiseModel(model, Dim), j1, j2),
        T12_(T12),
        T2hat_H_T1_(T12.inverse().AdjointMap()) {}

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void
  print(const std::string &s,
        const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "FrobeniusBetweenFactor<" << demangle(typeid(T).name())
              << ">(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    traits<T>::Print(T12_, "  T12: ");
    this->noiseModel_->print("  noise model: ");
  }

  /// assert equality up to a tolerance
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override {
    auto e = dynamic_cast<const FrobeniusBetweenFactor *>(&expected);
    return e != nullptr && NoiseModelFactorN<T, T>::equals(*e, tol) &&
           traits<T>::Equals(this->T12_, e->T12_, tol);
  }

  /// @}
  /// @name NoiseModelFactorN methods 
  /// @{

  /// Error is Frobenius norm between T1*T12 and T2.
  Vector evaluateError(const T& T1, const T& T2,
                       OptionalMatrixType H1, OptionalMatrixType H2) const override {
    const T T2hat = T1.compose(T12_);
    Eigen::Matrix<double, Dim, T::dimension> vec_H_T2hat;
    Vector error = T2.vec(H2) - T2hat.vec(H1 ? &vec_H_T2hat : nullptr);
    if (H1) *H1 = -vec_H_T2hat * T2hat_H_T1_;
    return error;
  }
  /// @}
};

}  // namespace gtsam
