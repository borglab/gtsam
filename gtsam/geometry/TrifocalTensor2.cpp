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

#include <stdexcept>
#include <vector>

namespace gtsam {

// Convert bearing measurements to projective coordinates.
std::vector<Point2> convertToProjective(const std::vector<Rot2>& rotations) {
  std::vector<Point2> projectives;
  projectives.reserve(rotations.size());
  for (const Rot2& rotation : rotations) {
    projectives.emplace_back(rotation.c() / rotation.s(), 1.0);
  }
  return projectives;
}

// Construct from 8 bearing measurements.
TrifocalTensor2 TrifocalTensor2::FromBearingMeasurements(
    const std::vector<Rot2>& bearings_u, const std::vector<Rot2>& bearings_v,
    const std::vector<Rot2>& bearings_w) {
  return TrifocalTensor2::FromProjectiveBearingMeasurements(
      convertToProjective(bearings_u), convertToProjective(bearings_v),
      convertToProjective(bearings_w));
}

// Construct from 8 bearing measurements expressed in projective coordinates.
TrifocalTensor2 TrifocalTensor2::FromProjectiveBearingMeasurements(
    const std::vector<Point2>& u, const std::vector<Point2>& v,
    const std::vector<Point2>& w) {
  if (u.size() < 8) {
    throw std::invalid_argument(
        "Trifocal tensor computation requires at least 8 measurements");
  }
  if (u.size() != v.size() || v.size() != w.size()) {
    throw std::invalid_argument(
        "Number of input measurements in 3 cameras must be same");
  }

  // Create the system matrix A.
  Matrix A(u.size() > 8 ? u.size() : 8, 8);
  for (int row = 0; row < u.size(); row++) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          A(row, 4 * i + 2 * j + k) = u[row](i) * v[row](j) * w[row](k);
        }
      }
    }
  }
  for (int row = u.size() - 8; row < 0; row++) {
    for (int col = 0; col < 8; col++) {
      A(row, col) = 0;
    }
  }

  // Eigen vector of smallest singular value is the trifocal tensor.
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

}  // namespace gtsam