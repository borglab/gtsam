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

#include <gtsam_unstable/slam/FrobeniusFactor.h>

#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {

boost::shared_ptr<noiseModel::Isotropic> ConvertPose3NoiseModel(
    const SharedNoiseModel& model, size_t d, bool defaultToUnit) {
  double sigma = 1.0;
  if (model != nullptr) {
    if (model->dim() != 6) {
      if (!defaultToUnit)
        throw std::runtime_error("Can only convert Pose3 noise models");
    } else {
      auto sigmas = model->sigmas().head(3).eval();
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
    Key j1, Key j2, const SO3& R12, size_t p, const SharedNoiseModel& model)
    : NoiseModelFactor2<SOn, SOn>(ConvertPose3NoiseModel(model, p * p), j1, j2),
      M_(p, p) {
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
  gttic(FrobeniusWormholeFactorTL_evaluateError);

  // Calculate G matrix of vectorized generators
  // TODO(generalize)
  const size_t p = Q1.rows();
  assert(Q2.rows() == p);
  const size_t pp = p * p, d = p * (p - 1) / 2;
  Matrix G(pp, d);
  for (size_t j = 0; j < d; j++) {
    // TODO(frank): this can't be right. Think about fixed vs dynamic.
    const auto X = SOn::Hat(p, Eigen::VectorXd::Unit(d, j));
    G.col(j) = Eigen::Map<const Matrix>(X.data(), pp, 1);
  }

  Vector fQ2(pp), hQ1(pp);
  fQ2 = Q2.vec(H2);
  const Matrix Q1M = Q1.matrix() * M_;
  hQ1 << Eigen::Map<const Matrix>(Q1M.data(), pp, 1);

  // If asked, calculate Jacobian as (M \otimes Q1) * G
  if (H1) {
    Matrix MxO(pp, pp);
    for (size_t i = 0; i < p; i++)
      for (size_t j = 0; j < p; j++)
        MxO.block(i * p, j * p, p, p) = Q1 * M_(j, i);
    *H1 = -MxO * G;
  }

  return fQ2 - hQ1;
}

/* ************************************************************************* */
FrobeniusWormholeFactorPi::FrobeniusWormholeFactorPi(
    Key j1, Key j2, const SO3& R12, const SharedNoiseModel& model)
    : NoiseModelFactor2<SO4, SO4>(ConvertPose3NoiseModel(model, 9), j1, j2),
      R12_(R12),
      MR_H_M_(so3::Dcompose(R12_)) {}

/* ************************************************************************* */
Vector FrobeniusWormholeFactorPi::evaluateError(
    const SO4& Q1, const SO4& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactorPi_evaluateError);

  // The projection Jacobians are computed by passing H1 and H2:
  Vector9 fQ2, hQ1;
  fQ2 << Eigen::Map<const Vector9>(Q2.topLeft(H2).data());
  const Matrix3 Q1R12 = so3::compose(Q1.topLeft(H1), R12_);
  hQ1 << Eigen::Map<const Vector9>(Q1R12.data());
  // Implement chain rule by pre-multiplying with constant Dcompose.
  if (H1) *H1 = -MR_H_M_ * (*H1);
  return fQ2 - hQ1;
}

/* ************************************************************************* */
FrobeniusWormholeFactor::FrobeniusWormholeFactor(Key j1, Key j2, const SO3& R12,
                                                 const SharedNoiseModel& model)
    : NoiseModelFactor2<SO4, SO4>(ConvertPose3NoiseModel(model, 12), j1, j2) {
  M_ << R12, 0, 0, 0;
}

/* ************************************************************************* */
Vector FrobeniusWormholeFactor::evaluateError(
    const SO4& Q1, const SO4& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactor_evaluateError);

  // The projection Jacobians are computed by passing H1 and H2:
  Vector12 fQ2, hQ1;
  fQ2 << Eigen::Map<const Vector12>(Q2.stiefel(H2).data());
  const Matrix43 Q1M = Q1.matrix() * M_;
  hQ1 << Eigen::Map<const Vector12>(Q1M.data());
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
FrobeniusWormholeFactorP::FrobeniusWormholeFactorP(
    Key j1, Key j2, const SO3& R12, size_t p, const SharedNoiseModel& model)
    : NoiseModelFactor2<SOn, SOn>(ConvertPose3NoiseModel(model, p * 3), j1, j2),
      R12_(R12) {}

/* ************************************************************************* */
Vector FrobeniusWormholeFactorP::evaluateError(
    const SOn& Q1, const SOn& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactorP_evaluateError);

  const size_t p = Q1.rows();
  const size_t dim = p * 3;  // Stiefel manifold dimension
  assert(noiseModel()->dim() == dim);

  // Calculate G matrix of vectorized generators
  // TODO(generalize)
  assert(Q2.rows() == p);
  const size_t p2 = p * 2, pp = p * p, d = p * (p - 1) / 2;
  Matrix Z = zeros(p, p);
  Matrix G(pp, d);
  for (size_t j = 0; j < d; j++) {
    // TODO(frank): this can't be right. Think about fixed vs dynamic.
    const auto X = SOn::Hat(p, Eigen::VectorXd::Unit(d, j));
    G.col(j) = Eigen::Map<const Matrix>(X.data(), pp, 1);
  }

  Vector fQ2(dim), hQ1(dim);

  // Vectorize and extract only d leftmost columns, i.e. vec(Q2*P)
  assert(Q2.rows() == p && Q2.cols() == p && dim <= pp);
  fQ2 << Eigen::Map<const Matrix>(Q2.data(), dim, 1);

  // Vectorize Q1*P*R12
  const Matrix Q1PR12 = Q1.leftCols<3>() * R12_;
  assert(Q1PR12.rows() == p && Q1PR12.cols() == 3);
  hQ1 << Eigen::Map<const Matrix>(Q1PR12.data(), dim, 1);

  // If asked, calculate Jacobian as (M \otimes Q1) * G
  if (H1) {
    Matrix RPxQ = zeros(dim, pp);
    RPxQ.block(0, 0, p, dim) << Q1 * R12_(0, 0), Q1 * R12_(1, 0),
        Q1 * R12_(2, 0);
    RPxQ.block(p, 0, p, dim) << Q1 * R12_(0, 1), Q1 * R12_(1, 1),
        Q1 * R12_(2, 1);
    RPxQ.block(p2, 0, p, dim) << Q1 * R12_(0, 2), Q1 * R12_(1, 2),
        Q1 * R12_(2, 2);
    *H1 = -RPxQ * G;
  }
  if (H2) {
    Matrix PxQ = zeros(dim, pp);
    PxQ.block(0, 0, p, p) = Q2;
    PxQ.block(p, p, p, p) = Q2;
    PxQ.block(p2, p2, p, p) = Q2;
    *H2 = PxQ * G;
  }

  return fQ2 - hQ1;
}

/* ************************************************************************* */

}  // namespace gtsam
