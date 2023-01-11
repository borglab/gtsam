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
 * error.
 * If the noise model is a robust error model, we use the sigmas of the
 * underlying noise model.
 *
 * If defaultToUnit == false throws an exception on unexepcted input.
 */
GTSAM_EXPORT SharedNoiseModel
ConvertNoiseModel(const SharedNoiseModel &model, size_t n,
                  bool defaultToUnit = true);

/**
 * FrobeniusPrior calculates the Frobenius norm between a given matrix and an
 * element of SO(3) or SO(4).
 */
template <class Rot>
class FrobeniusPrior : public NoiseModelFactorN<Rot> {
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };
  using MatrixNN = typename Rot::MatrixNN;
  Eigen::Matrix<double, Dim, 1> vecM_;  ///< vectorized matrix to approximate

 public:

  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor1<Rot>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor
  FrobeniusPrior(Key j, const MatrixNN& M,
                 const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<Rot>(ConvertNoiseModel(model, Dim), j) {
    vecM_ << Eigen::Map<const Matrix>(M.data(), Dim, 1);
  }

  /// Error is just Frobenius norm between Rot element and vectorized matrix M.
  Vector evaluateError(const Rot& R, OptionalMatrixType H) const override {
    return R.vec(H) - vecM_;  // Jacobian is computed only when needed.
  }
};

/**
 * FrobeniusFactor calculates the Frobenius norm between rotation matrices.
 * The template argument can be any fixed-size SO<N>.
 */
template <class Rot>
class FrobeniusFactor : public NoiseModelFactorN<Rot, Rot> {
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };

 public:

  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<Rot, Rot>::evaluateError;

  /// Constructor
  FrobeniusFactor(Key j1, Key j2, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<Rot, Rot>(ConvertNoiseModel(model, Dim), j1,
                                    j2) {}

  /// Error is just Frobenius norm between rotation matrices.
  Vector evaluateError(const Rot& R1, const Rot& R2,
                       OptionalMatrixType H1, OptionalMatrixType H2) const override {
    Vector error = R2.vec(H2) - R1.vec(H1);
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
template <class Rot>
class FrobeniusBetweenFactor : public NoiseModelFactorN<Rot, Rot> {
  Rot R12_;  ///< measured rotation between R1 and R2
  Eigen::Matrix<double, Rot::dimension, Rot::dimension>
      R2hat_H_R1_;  ///< fixed derivative of R2hat wrpt R1
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<Rot, Rot>::evaluateError;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructor
  /// @{

  /// Construct from two keys and measured rotation
  FrobeniusBetweenFactor(Key j1, Key j2, const Rot& R12,
                         const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<Rot, Rot>(
            ConvertNoiseModel(model, Dim), j1, j2),
        R12_(R12),
        R2hat_H_R1_(R12.inverse().AdjointMap()) {}

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void
  print(const std::string &s,
        const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "FrobeniusBetweenFactor<" << demangle(typeid(Rot).name())
              << ">(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    traits<Rot>::Print(R12_, "  R12: ");
    this->noiseModel_->print("  noise model: ");
  }

  /// assert equality up to a tolerance
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override {
    auto e = dynamic_cast<const FrobeniusBetweenFactor *>(&expected);
    return e != nullptr && NoiseModelFactorN<Rot, Rot>::equals(*e, tol) &&
           traits<Rot>::Equals(this->R12_, e->R12_, tol);
  }

  /// @}
  /// @name NoiseModelFactorN methods 
  /// @{

  /// Error is Frobenius norm between R1*R12 and R2.
  Vector evaluateError(const Rot& R1, const Rot& R2,
                       OptionalMatrixType H1, OptionalMatrixType H2) const override {
    const Rot R2hat = R1.compose(R12_);
    Eigen::Matrix<double, Dim, Rot::dimension> vec_H_R2hat;
    Vector error = R2.vec(H2) - R2hat.vec(H1 ? &vec_H_R2hat : nullptr);
    if (H1) *H1 = -vec_H_R2hat * R2hat_H_R1_;
    return error;
  }
  /// @}
};

}  // namespace gtsam
