

#include <gtsam/geometry/TrifocalTensor2.h>

#include <iostream>

namespace gtsam {
TrifocalTensor2::TrifocalTensor2(const std::vector<Rot2>& bearings_u,
                                 const std::vector<Rot2>& bearings_v,
                                 const std::vector<Rot2>& bearings_w) {
  int i = 0;
  gtsam::Matrix(bearing_u.size(), 8) system_matrix;
  for (int i = 0; i < bearing_u.size(); i++) {
    system_matrix(i, 0) = bearings_u.c() * bearings_v.c() * bearings_w.c();
    system_matrix(i, 1) = bearings_u.c() * bearings_v.c() * bearings_w.s();
    system_matrix(i, 2) = bearings_u.c() * bearings_v.s() * bearings_w.c();
    system_matrix(i, 3) = bearings_u.c() * bearings_v.s() * bearings_w.s();
    system_matrix(i, 4) = bearings_u.s() * bearings_v.c() * bearings_w.c();
    system_matrix(i, 5) = bearings_u.s() * bearings_v.c() * bearings_w.s();
    system_matrix(i, 6) = bearings_u.s() * bearings_v.s() * bearings_w.c();
    system_matrix(i, 7) = bearings_u.s() * bearings_v.s() * bearings_w.s();
  }

  gtsam::Matrix U, S, V;
  svd(system_matrix, U, S, V);
  gtsam::Vector8 V_last_column = column(V, V.cols() - 1);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      matrix0_(i, j) = V_last_column(2 * i + j);
      matrix1_(i, j) = V_last_column(2 * i + j + 4);
    }
  }
}

Rot2 transform(const Rot2& vZp, const Rot2& wZp) const{
    Rot2 uZp;
    Vector2 v_measurement, w_measurement;
    v_measurement << vZp.c(), vZp.s();
    w_measurement << wZp.c(), wZp.s();
    uZp = Rot2.atan(trans(v_measurement) * matrix0_ * w_measurement, - trans(v_measurement) * matrix1_ * w_measurement);
    return uZp;

}
}  // namespace gtsam