#include <gtsam/geometry/Line3.h>

namespace gtsam {

Line3 Line3::retract(const Vector4 &v, OptionalJacobian<4, 4> Dp, OptionalJacobian<4, 4> Dv) const {
  Vector3 w;
  w << v[0], v[1], 0;
  Rot3 incR;

  if (Dp) {
    Dp->setIdentity();
    incR = Rot3::Expmap(w);
    Dp->block<2, 2>(0, 0) = ((incR.matrix()).transpose()).block<2, 2>(0, 0);
  }
  if (Dv) {
    Matrix3 Dw;
    incR = Rot3::Expmap(w, Dw);
    Dv->setIdentity();
    Dv->block<2, 2>(0, 0) = Dw.block<2, 2>(0, 0);
  } else {
    incR = Rot3::Expmap(w);
  }
  Rot3 Rt = R_ * incR;
  return Line3(Rt, a_ + v[2], b_ + v[3]);
}

Vector4 Line3::localCoordinates(const Line3 &q, OptionalJacobian<4, 4> Dp,
                                OptionalJacobian<4, 4> Dq) const {
  Vector3 omega;
  Matrix3 D_log;
  omega = Rot3::Logmap(R_.inverse() * q.R_, D_log);
  if (Dp) {
    Matrix3 D_log_wp = -((q.R_).matrix()).transpose() * R_.matrix();
    Matrix3 Dwp = D_log * D_log_wp;
    Dp->setIdentity();
    Dp->block<2, 2>(0, 0) = Dwp.block<2, 2>(0, 0);
    (*Dp)(2, 2) = -1;
    (*Dp)(3, 3) = -1;
  }
  if (Dq) {
    Dq->setIdentity();
    Dq->block<2, 2>(0, 0) = D_log.block<2, 2>(0, 0);
  }
  Vector4 local;
  local << omega[0], omega[1], q.a_ - a_, q.b_ - b_;
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

Unit3 Line3::project(OptionalJacobian<2, 4> Dline) const {
  Vector3 V_0;
  V_0 << -b_, a_, 0.0;

  Unit3 l;
  if (Dline) {
    // Jacobian of the normalized Unit3 projected line with respect to
    // un-normalized Vector3 projected line in homogeneous coordinates.
    Matrix23 D_unit_line;
    l = Unit3::FromPoint3(Point3(R_ * V_0), D_unit_line);
    // Jacobian of the un-normalized Vector3 line with respect to
    // input 3D line
    Matrix34 D_vec_line = Matrix34::Zero();
    D_vec_line.col(0) = a_ * R_.r3();
    D_vec_line.col(1) = b_ * R_.r3();
    D_vec_line.col(2) = R_.r2();
    D_vec_line.col(3) = -R_.r1();
    // Jacobian of output wrt input is the product of the two.
    *Dline = D_unit_line * D_vec_line;
  } else {
    l = Unit3::FromPoint3(Point3(R_ * V_0));
  }
  return l;
}

Point3 Line3::point(double distance) const {
  // defining "center" of the line to be the point where it
  // intersects rotated XY axis
  Point3 center(a_, b_, 0);
  Point3 rotated_center = R_ * center;
  return rotated_center + distance * R_.r3();
}

Line3 transformTo(const Pose3 &wTc, const Line3 &wL,
                  OptionalJacobian<4, 6> Dpose, OptionalJacobian<4, 4> Dline) {
  Rot3 wRc = wTc.rotation();
  Rot3 cRw = wRc.inverse();
  Rot3 cRl = cRw * wL.R_;

  Vector2 w_ab;
  Vector3 t = ((wL.R_).transpose() * wTc.translation());
  Vector2 c_ab(wL.a_ - t[0], wL.b_ - t[1]);

  if (Dpose) {
    Matrix3 lRc = (cRl.matrix()).transpose();
    Dpose->setZero();
    // rotation
    Dpose->block<2, 3>(0, 0) = -lRc.block<2, 3>(0, 0);
    // translation
    Dpose->block<2, 3>(2, 3) = -lRc.block<2, 3>(0, 0);
  }
  if (Dline) {
    Dline->setIdentity();
    (*Dline)(0, 3) = -t[2];
    (*Dline)(1, 2) = t[2];
  }
  return Line3(cRl, c_ab[0], c_ab[1]);
}

}