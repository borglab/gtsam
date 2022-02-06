#include <gtsam/geometry/TrifocalTensor2.h>

#include <iostream>

namespace gtsam {

std::vector<Point2> convertToProjective(const std::vector<Rot2>& rotations) {
  std::vector<Point2> projectives;
  projectives.reserve(rotations.size());
  for (const Rot2& rotation : rotations) {
    projectives.emplace_back(rotation.c() / rotation.s(), 1.0);
  }
  return projectives;
}

TrifocalTensor2::TrifocalTensor2(const std::vector<Rot2>& bearings_u,
                                 const std::vector<Rot2>& bearings_v,
                                 const std::vector<Rot2>& bearings_w)
    : TrifocalTensor2(convertToProjective(bearings_u),
                      convertToProjective(bearings_v),
                      convertToProjective(bearings_w)) {}

TrifocalTensor2::TrifocalTensor2(const std::vector<Point2>& u,
                                 const std::vector<Point2>& v,
                                 const std::vector<Point2>& w) {
  if (u.size() != v.size() || v.size() != w.size()) {
    // throw error here
  }
  /*
  std::cout << "u, v, w:" << std::endl;
  for (int row = 0; row < u.size(); row++) {
    std::cout << u[row][0] << " " << u[row][1] << " " << v[row][0] << " "
              << v[row][1] << " " << w[row][0] << " " << w[row][1] << std::endl;
  }*/
  Matrix A(u.size() > 8? u.size():8, 8);
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
  for (int row = u.size() - 8; row < 0; row ++){
    for (int col = 0; col < 8; col ++){
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