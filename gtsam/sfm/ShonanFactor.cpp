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

#include <gtsam/sfm/ShonanFactor.h>

#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {

//******************************************************************************
template <size_t d>
ShonanFactor<d>::ShonanFactor(Key j1, Key j2, const Rot &R12, size_t p,
                              const SharedNoiseModel &model,
                              const boost::shared_ptr<Matrix> &G)
    : NoiseModelFactor2<SOn, SOn>(ConvertNoiseModel(model, p * d), j1, j2),
      M_(R12.matrix()), // d*d in all cases
      p_(p),            // 4 for SO(4)
      pp_(p * p),       // 16 for SO(4)
      G_(G) {
  if (noiseModel()->dim() != d * p_)
    throw std::invalid_argument(
        "ShonanFactor: model with incorrect dimension.");
  if (!G) {
    G_ = boost::make_shared<Matrix>();
    *G_ = SOn::VectorizedGenerators(p); // expensive!
  }
  if (static_cast<size_t>(G_->rows()) != pp_ ||
      static_cast<size_t>(G_->cols()) != SOn::Dimension(p))
    throw std::invalid_argument("ShonanFactor: passed in generators "
                                "of incorrect dimension.");
}

//******************************************************************************
template <size_t d>
void ShonanFactor<d>::print(const std::string &s,
                            const KeyFormatter &keyFormatter) const {
  std::cout << s << "ShonanFactor<" << p_ << ">(" << keyFormatter(key1()) << ","
            << keyFormatter(key2()) << ")\n";
  traits<Matrix>::Print(M_, "  M: ");
  noiseModel_->print("  noise model: ");
}

//******************************************************************************
template <size_t d>
bool ShonanFactor<d>::equals(const NonlinearFactor &expected,
                             double tol) const {
  auto e = dynamic_cast<const ShonanFactor *>(&expected);
  return e != nullptr && NoiseModelFactor2<SOn, SOn>::equals(*e, tol) &&
         p_ == e->p_ && M_ == e->M_;
}

//******************************************************************************
template <size_t d>
void ShonanFactor<d>::fillJacobians(const Matrix &M1, const Matrix &M2,
                                    boost::optional<Matrix &> H1,
                                    boost::optional<Matrix &> H2) const {
  gttic(ShonanFactor_Jacobians);
  const size_t dim = p_ * d; // Stiefel manifold dimension

  if (H1) {
    // If asked, calculate Jacobian H1 as as (M' \otimes M1) * G
    // M' = dxd, M1 = pxp, G = (p*p)xDim(p), result should be dim x Dim(p)
    // (M' \otimes M1) is dim*dim, but last pp-dim columns are zero
    *H1 = Matrix::Zero(dim, G_->cols());
    for (size_t j = 0; j < d; j++) {
      auto MG_j = M1 * G_->middleRows(j * p_, p_); // p_ * Dim(p)
      for (size_t i = 0; i < d; i++) {
        H1->middleRows(i * p_, p_) -= M_(j, i) * MG_j;
      }
    }
  }
  if (H2) {
    // If asked, calculate Jacobian H2 as as (I_d \otimes M2) * G
    // I_d = dxd, M2 = pxp, G = (p*p)xDim(p), result should be dim x Dim(p)
    // (I_d \otimes M2) is dim*dim, but again last pp-dim columns are zero
    H2->resize(dim, G_->cols());
    for (size_t i = 0; i < d; i++) {
      H2->middleRows(i * p_, p_) = M2 * G_->middleRows(i * p_, p_);
    }
  }
}

//******************************************************************************
template <size_t d>
Vector ShonanFactor<d>::evaluateError(const SOn &Q1, const SOn &Q2,
                                      boost::optional<Matrix &> H1,
                                      boost::optional<Matrix &> H2) const {
  gttic(ShonanFactor_evaluateError);

  const Matrix &M1 = Q1.matrix();
  const Matrix &M2 = Q2.matrix();
  if (M1.rows() != static_cast<int>(p_) || M2.rows() != static_cast<int>(p_))
    throw std::invalid_argument("Invalid dimension SOn values passed to "
                                "ShonanFactor<d>::evaluateError");

  const size_t dim = p_ * d; // Stiefel manifold dimension
  Vector fQ2(dim), hQ1(dim);

  // Vectorize and extract only d leftmost columns, i.e. vec(M2*P)
  fQ2 << Eigen::Map<const Matrix>(M2.data(), dim, 1);

  // Vectorize M1*P*R12
  const Matrix Q1PR12 = M1.leftCols<d>() * M_;
  hQ1 << Eigen::Map<const Matrix>(Q1PR12.data(), dim, 1);

  this->fillJacobians(M1, M2, H1, H2);

  return fQ2 - hQ1;
}

/* ************************************************************************* */
// Explicit instantiation for d=2 and d=3
template class ShonanFactor<2>;
template class ShonanFactor<3>;

//******************************************************************************

} // namespace gtsam
