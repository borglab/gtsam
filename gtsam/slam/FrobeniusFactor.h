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

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * When creating (any) FrobeniusFactor we convert a 6-dimensional Pose3
 * BetweenFactor noise model into an 9 or 16-dimensional isotropic noise
 * model used to weight the Frobenius norm.  If the noise model passed is
 * null we return a Dim-dimensional isotropic noise model with sigma=1.0. If
 * not, we we check if the 3-dimensional noise model on rotations is
 * isotropic. If it is, we extend to 'Dim' dimensions, otherwise we throw an
 * error. If defaultToUnit == false throws an exception on unexepcted input.
 */
  GTSAM_EXPORT boost::shared_ptr<noiseModel::Isotropic> ConvertPose3NoiseModel(
    const SharedNoiseModel& model, size_t d, bool defaultToUnit = true);

/**
 * FrobeniusPrior calculates the Frobenius norm between a given matrix and an
 * element of SO(3) or SO(4).
 */
template <class Rot>
class FrobeniusPrior : public NoiseModelFactor1<Rot> {
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };
  using MatrixNN = typename Rot::MatrixNN;
  Eigen::Matrix<double, Dim, 1> vecM_;  ///< vectorized matrix to approximate

 public:
  /// Constructor
  FrobeniusPrior(Key j, const MatrixNN& M,
                 const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor1<Rot>(ConvertPose3NoiseModel(model, Dim), j) {
    vecM_ << Eigen::Map<const Matrix>(M.data(), Dim, 1);
  }

  /// Error is just Frobenius norm between Rot element and vectorized matrix M.
  Vector evaluateError(const Rot& R,
                       boost::optional<Matrix&> H = boost::none) const {
    return R.vec(H) - vecM_;  // Jacobian is computed only when needed.
  }
};

/**
 * FrobeniusFactor calculates the Frobenius norm between rotation matrices.
 * The template argument can be any fixed-size SO<N>.
 */
template <class Rot>
class FrobeniusFactor : public NoiseModelFactor2<Rot, Rot> {
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };

 public:
  /// Constructor
  FrobeniusFactor(Key j1, Key j2, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor2<Rot, Rot>(ConvertPose3NoiseModel(model, Dim), j1,
                                    j2) {}

  /// Error is just Frobenius norm between rotation matrices.
  Vector evaluateError(const Rot& R1, const Rot& R2,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const {
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
class FrobeniusBetweenFactor : public NoiseModelFactor2<Rot, Rot> {
  Rot R12_;  ///< measured rotation between R1 and R2
  Eigen::Matrix<double, Rot::dimension, Rot::dimension>
      R2hat_H_R1_;  ///< fixed derivative of R2hat wrpt R1
  enum { Dim = Rot::VectorN2::RowsAtCompileTime };

 public:
  /// Constructor
  FrobeniusBetweenFactor(Key j1, Key j2, const Rot& R12,
                         const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor2<Rot, Rot>(
            ConvertPose3NoiseModel(model, Dim), j1, j2),
        R12_(R12),
        R2hat_H_R1_(R12.inverse().AdjointMap()) {}

  /// Error is Frobenius norm between R1*R12 and R2.
  Vector evaluateError(const Rot& R1, const Rot& R2,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const {
    const Rot R2hat = R1.compose(R12_);
    Eigen::Matrix<double, Dim, Rot::dimension> vec_H_R2hat;
    Vector error = R2.vec(H2) - R2hat.vec(H1 ? &vec_H_R2hat : nullptr);
    if (H1) *H1 = -vec_H_R2hat * R2hat_H_R1_;
    return error;
  }
};

/**
 * FrobeniusWormholeFactor is a BetweenFactor that moves in SO(p), but will
 * land on the SO(3) sub-manifold of SO(p) at the global minimum. It projects
 * the SO(p) matrices down to a Stiefel manifold of p*d matrices.
 * TODO(frank): template on D=2 or 3
 */
class GTSAM_EXPORT FrobeniusWormholeFactor : public NoiseModelFactor2<SOn, SOn> {
  Matrix M_;                   ///< measured rotation between R1 and R2
  size_t p_, pp_, dimension_;  ///< dimensionality constants
  Matrix G_;                   ///< matrix of vectorized generators

 public:
  /// Constructor. Note we convert to 3*p-dimensional noise model.
  FrobeniusWormholeFactor(Key j1, Key j2, const Rot3& R12, size_t p = 4,
                          const SharedNoiseModel& model = nullptr);

  /// Error is Frobenius norm between Q1*P*R12 and Q2*P, where P=[I_3x3;0]
  /// projects down from SO(p) to the Stiefel manifold of px3 matrices.
  Vector evaluateError(const SOn& Q1, const SOn& Q2,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const;
};

}  // namespace gtsam
