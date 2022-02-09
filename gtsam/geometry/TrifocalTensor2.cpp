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
#include <iostream>

namespace gtsam {
// Convert bearing measurements to projective form
std::vector<Point2> convertToProjective(const std::vector<Rot2>& rotations) {
  std::vector<Point2> projectives;
  projectives.reserve(rotations.size());
  for (const Rot2& rotation : rotations) {
    projectives.emplace_back(rotation.c() / rotation.s(), 1.0);
  }
  return projectives;
}

// Construct from 8 bearing measurements.
TrifocalTensor2::TrifocalTensor2(const std::vector<Rot2>& bearings_u,
                                 const std::vector<Rot2>& bearings_v,
                                 const std::vector<Rot2>& bearings_w)
    : TrifocalTensor2(convertToProjective(bearings_u),
                      convertToProjective(bearings_v),
                      convertToProjective(bearings_w)) {}

// Construct from 8 bearing measurements expressed in projective coordinates.
TrifocalTensor2::TrifocalTensor2(const std::vector<Point2>& u,
                                 const std::vector<Point2>& v,
                                 const std::vector<Point2>& w) {
  if (u.size() != v.size() || v.size() != w.size()) {
    // throw error here
  }

  Matrix A(u.size() > 8 ? u.size() : 8, 8);
  std::cout << "system matrix:" << std::endl;
  for (int row = 0; row < u.size(); row++) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          A(row, 4 * i + 2 * j + k) = u[row](i) * v[row](j) * w[row](k);
          std::cout << A(row, 4 * i + 2 * j + k) << " ";
        }
      }
    }
    std::cout << std::endl;
  }
  for (int row = u.size() - 8; row < 0; row++) {
    for (int col = 0; col < 8; col++) {
      A(row, col) = 0;
      std::cout << A(row, col) << " ";
    }
    std::cout << std::endl;
  }
  Matrix U, V;
  Vector S;
  svd(A, U, S, V);
  std::cout << "V:" << std::endl;
  for (int i = 0; i < V.rows(); i++) {
    for (int j = 0; j < V.cols(); j++) {
      std::cout << V(i, j) << " ";
    }
    std::cout << std::endl;
  }
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      matrix0_(i, j) = V(2 * i + j, V.cols() - 1);
      matrix1_(i, j) = V(2 * i + j + 4, V.cols() - 1);
    }
  }
}

// Finds a measurement in the first view using measurements from second and
// third views.
Rot2 TrifocalTensor2::transform(const Rot2& vZp, const Rot2& wZp) const {
  Rot2 uZp;
  Vector2 v_measurement, w_measurement;
  v_measurement << vZp.c(), vZp.s();
  w_measurement << wZp.c(), wZp.s();
  return Rot2::atan2(dot(matrix0_ * w_measurement, v_measurement),
                     -dot(matrix1_ * w_measurement, v_measurement));
}

Matrix2 TrifocalTensor2::mat0() const { return matrix0_; }
Matrix2 TrifocalTensor2::mat1() const { return matrix1_; }

}  // namespace gtsam