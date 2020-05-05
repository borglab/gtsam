#include <gtsam/geometry/Line3.h>
#include <cstdlib>

namespace gtsam {

Line3 Line3::retract(const Vector4 &v, OptionalJacobian<4, 4> H) const {
  Vector3 w;
  w << v[0], v[1], 0;
  Rot3 eps;
  if (H) {
    OptionalJacobian<3, 3> Dw;
    eps = Rot3::Expmap(w, Dw);
    H->block<2, 2>(0, 0) = Dw->block<2, 2>(0, 0);
    (*H)(2, 2) = 1;
    (*H)(3, 3) = 1;
  } else {
    eps = Rot3::Expmap(w);
  }
  Rot3 Rt = R_ * eps;
  return Line3(Rt, a_ + v[2], b_ + v[3]);
}

Vector4 Line3::localCoordinates(const Line3 &q, OptionalJacobian<4, 4> H) const {
  Vector3 local_rot;
  Vector4 local;
  Vector2 ab_q(q.V());
  if (H) {
    OptionalJacobian<3, 3> Dw;
    local_rot = Rot3::Logmap(R_.inverse() * q.R(), Dw);
    H->block<2, 2>(0, 0) = Dw->block<2, 2>(0, 0);
    (*H)(2, 2) = 1;
    (*H)(3, 3) = 1;
  } else {
    local_rot = Rot3::Logmap(R_.inverse() * q.R());
  }
  local << local_rot[0], local_rot[1], ab_q[0] - a_, ab_q[1] - b_;
  return local;
}

void Line3::print(const std::string &s) const {
  std::cout << s << std::endl;
  R_.print("R:\n");
  std::cout << "a: " << a_ << ", b: " << b_ << std::endl;
}

bool Line3::equals(const Line3 &l2, double tol) const {
  Vector4 diff = localCoordinates(l2);
  return fabs(diff[0]) < tol && fabs(diff[1]) < tol
      && fabs(diff[2]) < tol && fabs(diff[3]) < tol;
}

Point3 Line3::project(OptionalJacobian<3, 4> Dline) const {
  Vector3 V_0;
  V_0 << -b_, a_, 0;

  Unit3 l = R_ * V_0;
  if (Dline) {
    Dline->setZero();
    Dline->col(0) = a_ * R_.r3();
    Dline->col(1) = b_ * R_.r3();
    Dline->col(2) = R_.r2();
    Dline->col(3) = -R_.r1();
  }
  return l;
}

Line3 transformTo(const Pose3 &wTc, const Line3 &wL,
                  OptionalJacobian<4, 6> Dpose, OptionalJacobian<4, 4> Dline) {
  Rot3 wRl = wL.R();
  Rot3 wRc = wTc.rotation();
  Rot3 cRw = wRc.inverse();
  Rot3 cRl = cRw * wRl;

  Vector2 w_ab = wL.V();
  Vector3 t = (wRl.transpose() * wTc.translation());
  Vector2 c_ab(w_ab[0] - t[0], w_ab[1] - t[1]);

  if (Dpose) {
    // translation due to translation
    Matrix3 cRl_mat = cRl.matrix();
    Matrix3 lRc = cRl_mat.transpose();
    Dpose->block<1, 3>(2, 3) = -lRc.row(0);
    Dpose->block<1, 3>(3, 3) = -lRc.row(1);

    Dpose->block<1, 3>(0, 0) = -lRc.row(0);
    Dpose->block<1, 3>(1, 0) = -lRc.row(1);
  }
  if (Dline) {
    Dline->col(0) << 1.0, 0.0, 0.0, -t[2];
    Dline->col(1) << 0.0, 1.0, t[2], 0.0;
    Dline->col(2) << 0.0, 0.0, 1.0, 0.0;
    Dline->col(3) << 0.0, 0.0, 0.0, 1.0;
  }
  return Line3(cRl, c_ab[0], c_ab[1]);
}

}
