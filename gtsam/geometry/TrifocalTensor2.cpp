/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    TrifocalTensor2.cpp
 * @brief   A 2x2x2 trifocal tensor in a plane, for 1D cameras.
 * @author  Zhaodong Yang
 * @author  Akshay Krishnan
 */

#include <gtsam/geometry/TrifocalTensor2.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace gtsam {
namespace {

// Converts bearings without dividing by a potentially zero sine component.
std::vector<Point2> convertToProjective(const std::vector<Rot2>& rotations) {
  std::vector<Point2> projectives;
  projectives.reserve(rotations.size());
  for (const Rot2& rotation : rotations) {
    projectives.emplace_back(rotation.c(), rotation.s());
  }
  return projectives;
}

}  // namespace

/* ************************************************************************* */
TrifocalTensor2 TrifocalTensor2::FromBearingMeasurements(
    const std::vector<Rot2>& bearings_u, const std::vector<Rot2>& bearings_v,
    const std::vector<Rot2>& bearings_w) {
  return TrifocalTensor2::FromProjectiveBearingMeasurements(
      convertToProjective(bearings_u), convertToProjective(bearings_v),
      convertToProjective(bearings_w));
}

/* ************************************************************************* */
TrifocalTensor2 TrifocalTensor2::FromProjectiveBearingMeasurements(
    const std::vector<Point2>& u, const std::vector<Point2>& v,
    const std::vector<Point2>& w) {
  if (u.size() != v.size() || v.size() != w.size()) {
    throw std::invalid_argument(
        "Trifocal tensor inputs must have the same number of measurements");
  }
  if (u.size() < 7) {
    throw std::invalid_argument(
        "Trifocal tensor computation requires at least 7 measurements");
  }

  // Seven measurements yield a 7x8 system. Add one zero row so that the full
  // right-singular-vector basis contains the one-dimensional nullspace.
  Matrix A = Matrix::Zero(std::max(u.size(), size_t{8}), 8);
  for (size_t row = 0; row < u.size(); ++row) {
    for (size_t i = 0; i < 2; ++i) {
      for (size_t j = 0; j < 2; ++j) {
        for (size_t k = 0; k < 2; ++k) {
          A(row, 4 * i + 2 * j + k) = u[row](i) * v[row](j) * w[row](k);
        }
      }
    }
  }

  // The right singular vector of smallest singular value is the tensor.
  Matrix U, V;
  Vector S;
  svd(A, U, S, V);

  Matrix2 matrix0, matrix1;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      matrix0(i, j) = V(2 * i + j, V.cols() - 1);
      matrix1(i, j) = V(2 * i + j + 4, V.cols() - 1);
    }
  }
  return TrifocalTensor2(matrix0, matrix1);
}

/* ************************************************************************* */
Rot2 TrifocalTensor2::transform(const Rot2& vZp, const Rot2& wZp) const {
  const Point2 projective =
      transform(Point2(vZp.c(), vZp.s()), Point2(wZp.c(), wZp.s()));
  return Rot2::atan2(projective.y(), projective.x());
}

/* ************************************************************************* */
Point2 TrifocalTensor2::transform(const Point2& vZp, const Point2& wZp) const {
  const double coefficient0 = dot(matrix0_ * wZp, vZp);
  const double coefficient1 = dot(matrix1_ * wZp, vZp);
  return Point2(-coefficient1, coefficient0);
}

/* ************************************************************************* */
void TrifocalTensor2::print(const std::string& s) const {
  std::cout << s << "matrix0:\n"
            << matrix0_ << "\nmatrix1:\n"
            << matrix1_ << std::endl;
}

/* ************************************************************************* */
bool TrifocalTensor2::equals(const TrifocalTensor2& other, double tol) const {
  const double norm =
      std::sqrt(matrix0_.squaredNorm() + matrix1_.squaredNorm());
  const double otherNorm =
      std::sqrt(other.matrix0_.squaredNorm() + other.matrix1_.squaredNorm());

  if (norm <= tol || otherNorm <= tol) {
    const double difference =
        std::sqrt((matrix0_ - other.matrix0_).squaredNorm() +
                  (matrix1_ - other.matrix1_).squaredNorm());
    return difference <= tol;
  }

  const Matrix2 normalized0 = matrix0_ / norm;
  const Matrix2 normalized1 = matrix1_ / norm;
  const Matrix2 otherNormalized0 = other.matrix0_ / otherNorm;
  const Matrix2 otherNormalized1 = other.matrix1_ / otherNorm;
  const double sameScale =
      std::sqrt((normalized0 - otherNormalized0).squaredNorm() +
                (normalized1 - otherNormalized1).squaredNorm());
  const double oppositeScale =
      std::sqrt((normalized0 + otherNormalized0).squaredNorm() +
                (normalized1 + otherNormalized1).squaredNorm());
  return std::min(sameScale, oppositeScale) <= tol;
}

}  // namespace gtsam
