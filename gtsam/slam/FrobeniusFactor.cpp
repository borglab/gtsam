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

#include <gtsam/slam/FrobeniusFactor.h>

#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {

//******************************************************************************
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

//******************************************************************************
FrobeniusWormholeFactor::FrobeniusWormholeFactor(Key j1, Key j2, const Rot3& R12,
                                                 size_t p,
                                                 const SharedNoiseModel& model)
    : NoiseModelFactor2<SOn, SOn>(ConvertPose3NoiseModel(model, p * 3), j1, j2),
      M_(R12.matrix()),               // 3*3 in all cases
      p_(p),                          // 4 for SO(4)
      pp_(p * p),                     // 16 for SO(4)
      dimension_(SOn::Dimension(p)),  // 6 for SO(4)
      G_(pp_, dimension_)             // 16*6 for SO(4)
{
  // Calculate G matrix of vectorized generators
  Matrix Z = Matrix::Zero(p, p);
  for (size_t j = 0; j < dimension_; j++) {
    const auto X = SOn::Hat(Eigen::VectorXd::Unit(dimension_, j));
    G_.col(j) = Eigen::Map<const Matrix>(X.data(), pp_, 1);
  }
  assert(noiseModel()->dim() == 3 * p_);
}

//******************************************************************************
Vector FrobeniusWormholeFactor::evaluateError(
    const SOn& Q1, const SOn& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactorP_evaluateError);

  const Matrix& M1 = Q1.matrix();
  const Matrix& M2 = Q2.matrix();
  assert(M1.rows() == p_ && M2.rows() == p_);

  const size_t dim = 3 * p_;  // Stiefel manifold dimension
  Vector fQ2(dim), hQ1(dim);

  // Vectorize and extract only d leftmost columns, i.e. vec(M2*P)
  fQ2 << Eigen::Map<const Matrix>(M2.data(), dim, 1);

  // Vectorize M1*P*R12
  const Matrix Q1PR12 = M1.leftCols<3>() * M_;
  hQ1 << Eigen::Map<const Matrix>(Q1PR12.data(), dim, 1);

  // If asked, calculate Jacobian as (M \otimes M1) * G
  if (H1) {
    const size_t p2 = 2 * p_;
    Matrix RPxQ = Matrix::Zero(dim, pp_);
    RPxQ.block(0, 0, p_, dim) << M1 * M_(0, 0), M1 * M_(1, 0), M1 * M_(2, 0);
    RPxQ.block(p_, 0, p_, dim) << M1 * M_(0, 1), M1 * M_(1, 1), M1 * M_(2, 1);
    RPxQ.block(p2, 0, p_, dim) << M1 * M_(0, 2), M1 * M_(1, 2), M1 * M_(2, 2);
    *H1 = -RPxQ * G_;
  }
  if (H2) {
    const size_t p2 = 2 * p_;
    Matrix PxQ = Matrix::Zero(dim, pp_);
    PxQ.block(0, 0, p_, p_) = M2;
    PxQ.block(p_, p_, p_, p_) = M2;
    PxQ.block(p2, p2, p_, p_) = M2;
    *H2 = PxQ * G_;
  }

  return fQ2 - hQ1;
}

//******************************************************************************

}  // namespace gtsam
