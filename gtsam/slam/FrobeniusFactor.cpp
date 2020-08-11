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
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {

//******************************************************************************
boost::shared_ptr<noiseModel::Isotropic>
ConvertNoiseModel(const SharedNoiseModel &model, size_t d, bool defaultToUnit) {
  double sigma = 1.0;
  if (model != nullptr) {
    auto sigmas = model->sigmas();
    size_t n = sigmas.size();
    if (n == 1) {
      sigma = sigmas(0); // Rot2
      goto exit;
    }
    if (n == 3 || n == 6) {
      sigma = sigmas(2); // Pose2, Rot3, or Pose3
      if (sigmas(0) != sigma || sigmas(1) != sigma) {
        if (!defaultToUnit) {
          throw std::runtime_error("Can only convert isotropic rotation noise");
        }
      }
      goto exit;
    }
    if (!defaultToUnit) {
      throw std::runtime_error("Can only convert Pose2/Pose3 noise models");
    }
  }
exit:
  return noiseModel::Isotropic::Sigma(d, sigma);
}

//******************************************************************************
template <size_t d>
FrobeniusWormholeFactor<d>::FrobeniusWormholeFactor(
    Key j1, Key j2, const Rot &R12, size_t p, const SharedNoiseModel &model,
    const boost::shared_ptr<Matrix> &G)
    : NoiseModelFactor2<SOn, SOn>(ConvertNoiseModel(model, p * 3), j1, j2),
      M_(R12.matrix()), // 3*3 in all cases
      p_(p),            // 4 for SO(4)
      pp_(p * p),       // 16 for SO(4)
      G_(G) {
  if (noiseModel()->dim() != 3 * p_)
    throw std::invalid_argument(
        "FrobeniusWormholeFactor: model with incorrect dimension.");
  if (!G) {
    G_ = boost::make_shared<Matrix>();
    *G_ = SOn::VectorizedGenerators(p); // expensive!
  }
  if (G_->rows() != pp_ || G_->cols() != SOn::Dimension(p))
    throw std::invalid_argument("FrobeniusWormholeFactor: passed in generators "
                                "of incorrect dimension.");
}

//******************************************************************************
template <size_t d>
void FrobeniusWormholeFactor<d>::print(const std::string &s, const KeyFormatter &keyFormatter) const {
  std::cout << s << "FrobeniusWormholeFactor<" << p_ << ">(" << keyFormatter(key1()) << ","
            << keyFormatter(key2()) << ")\n";
  traits<Matrix>::Print(M_, "  M: ");
  noiseModel_->print("  noise model: ");
}

//******************************************************************************
template <size_t d>
bool FrobeniusWormholeFactor<d>::equals(const NonlinearFactor &expected,
                                     double tol) const {
  auto e = dynamic_cast<const FrobeniusWormholeFactor *>(&expected);
  return e != nullptr && NoiseModelFactor2<SOn, SOn>::equals(*e, tol) &&
         p_ == e->p_ && M_ == e->M_;
}

//******************************************************************************
template <size_t d>
Vector FrobeniusWormholeFactor<d>::evaluateError(
    const SOn& Q1, const SOn& Q2, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  gttic(FrobeniusWormholeFactorP_evaluateError);

  const Matrix& M1 = Q1.matrix();
  const Matrix& M2 = Q2.matrix();
  if (M1.rows() != static_cast<int>(p_) || M2.rows() != static_cast<int>(p_))
    throw std::invalid_argument(
        "Invalid dimension SOn values passed to "
        "FrobeniusWormholeFactor<d>::evaluateError");

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
    *H1 = -RPxQ * (*G_);
  }
  if (H2) {
    const size_t p2 = 2 * p_;
    Matrix PxQ = Matrix::Zero(dim, pp_);
    PxQ.block(0, 0, p_, p_) = M2;
    PxQ.block(p_, p_, p_, p_) = M2;
    PxQ.block(p2, p2, p_, p_) = M2;
    *H2 = PxQ * (*G_);
  }

  return fQ2 - hQ1;
}

/* ************************************************************************* */
// Explicit instantiation for d=2 and d=3
template class FrobeniusWormholeFactor<2>;
template class FrobeniusWormholeFactor<3>;

//******************************************************************************

}  // namespace gtsam
