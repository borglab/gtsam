/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FrobeniusFactor.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Various factors that minimize some Frobenius norm
 */

#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/slam/FrobeniusFactor.h>

#include <iostream>
#include <vector>

using namespace std;

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
static boost::shared_ptr<noiseModel::Isotropic> ConverPose3NoiseModel(
    const gtsam::SharedNoiseModel& model, size_t d, bool defaultToUnit = true) {
  double sigma = 1.0;
  if (model != nullptr) {
    if (model->dim() != 6) {
      if (!defaultToUnit)
        throw std::runtime_error("Can only convert Pose3 noise models");
    } else {
      auto sigmas = model->sigmas().head(3);
      if (sigmas(1) != sigmas(0) || sigmas(2) != sigmas(0)) {
        if (!defaultToUnit)
          throw std::runtime_error("Can only convert isotropic rotation noise");
      } else {
        sigma = sigmas(0);
      }
    }
  }
  return noiseModel::Isotropic::Sigma(d, sigma);
}

/* ************************************************************************* */
FrobeniusWormholeFactorTL::FrobeniusWormholeFactorTL(
    Key j1, Key j2, const SO3& R12, size_t n,
    const gtsam::SharedNoiseModel& model)
    : gtsam::NoiseModelFactor2<SOn, SOn>(ConverPose3NoiseModel(model, n * n),
                                         j1, j2),
      M_(n, n) {
  M_.setZero();
  M_.topLeftCorner<3, 3>() = R12;
}

/* ************************************************************************* */
// TODO(frank): make SO(3) and SO(4) \bar{G} available
static SO4::Vector16 vec(const SO4& Q) {
  return Eigen::Map<const SO4::Vector16>(Q.data());
}

static const std::vector<const Matrix4> G(
    {SO4::Hat(Vector6::Unit(0)), SO4::Hat(Vector6::Unit(1)),
     SO4::Hat(Vector6::Unit(2)), SO4::Hat(Vector6::Unit(3)),
     SO4::Hat(Vector6::Unit(4)), SO4::Hat(Vector6::Unit(5))});

static const Eigen::Matrix<double, 16, 6> P =
    (Eigen::Matrix<double, 16, 6>() << vec(G[0]), vec(G[1]), vec(G[2]),
     vec(G[3]), vec(G[4]), vec(G[5]))
        .finished();

/* ************************************************************************* */
Vector FrobeniusWormholeFactorTL::evaluateError(
    const SOn& Q1, const SOn& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactor_evaluateError);

  // Calculate G matrix of vectorized generators
  // TODO(generalize)
  const size_t n = Q1.rows();
  assert(Q2.rows() == n);
  const size_t n2 = n * n, d = n * (n - 1) / 2;
  Matrix G(n2, d);
  for (size_t j = 0; j < d; j++) {
    // TODO(frank): this can't be right. Think about fixed vs dynamic.
    const auto X = SOn::Hat(n, Eigen::VectorXd::Unit(d, j));
    G.col(j) = Eigen::Map<const Matrix>(X.data(), n2, 1);
  }

  // << operator below implements vec
  Vector fQ2(n2), hQ1(n2);
  fQ2 = Q2.vec(H2);
  const Matrix Q1M = Q1.matrix() * M_;
  hQ1 << Eigen::Map<const Matrix>(Q1M.data(), n2, 1);

  // If asked, calculate Jacobian as (M \otimes Q1) * G
  if (H1) {
    Matrix MxO(n2, n2);
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < n; j++)
        MxO.block(i * n, j * n, n, n) = Q1 * M_(j, i);
    *H1 = -MxO * G;
  }

  return fQ2 - hQ1;
}

/* ************************************************************************* */
FrobeniusWormholeFactorPi::FrobeniusWormholeFactorPi(
    Key j1, Key j2, const SO3& R12, const gtsam::SharedNoiseModel& model)
    : gtsam::NoiseModelFactor2<SO4, SO4>(
          noiseModel::FrobeniusNoiseModel<9>::FromPose3NoiseModel(model), j1,
          j2),
      R12_(R12),
      MR_H_M_(so3::Dcompose(R12_)) {}

/* ************************************************************************* */
Vector FrobeniusWormholeFactorPi::evaluateError(
    const SO4& Q1, const SO4& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactor_evaluateError);

  // << operator below implements vec
  // The projection Jacobians are computed by passing H1 and H2:
  Vector9 fQ2, hQ1;
  fQ2 << Q2.topLeft(H2);
  hQ1 << so3::compose(Q1.topLeft(H1), R12_);
  // Implement chain rule by pre-multiplying with constant Dcompose.
  if (H1) *H1 = -MR_H_M_ * (*H1);
  return fQ2 - hQ1;
}

/* ************************************************************************* */
FrobeniusWormholeFactor::FrobeniusWormholeFactor(
    Key j1, Key j2, const SO3& R12, const gtsam::SharedNoiseModel& model)
    : gtsam::NoiseModelFactor2<SO4, SO4>(
          noiseModel::FrobeniusNoiseModel<12>::FromPose3NoiseModel(model), j1,
          j2) {
  M_ << R12, 0, 0, 0;
}

/* ************************************************************************* */
Vector FrobeniusWormholeFactor::evaluateError(
    const SO4& Q1, const SO4& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactor_evaluateError);

  // << operator below implements vec
  // The projection Jacobians are computed by passing H1 and H2:
  Vector12 fQ2, hQ1;
  fQ2 << Q2.stiefel(H2);
  hQ1 << Q1.matrix() * M_;
  // We do the combined derivative of Q*E'*R which is (R'E \otimes Q)P
  if (H1) {
    Eigen::Matrix<double, 12, 16> MxO;
    MxO << Q1 * M_(0, 0), Q1 * M_(1, 0), Q1 * M_(2, 0), Z_4x4,  //
        Q1 * M_(0, 1), Q1 * M_(1, 1), Q1 * M_(2, 1), Z_4x4,     //
        Q1 * M_(0, 2), Q1 * M_(1, 2), Q1 * M_(2, 2), Z_4x4;
    *H1 = -MxO * P;
  }
  return fQ2 - hQ1;
}

/* ************************************************************************* */

}  // namespace gtsam
