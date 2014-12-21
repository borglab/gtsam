/*
 * @file EssentialMatrix.cpp
 * @brief EssentialMatrix class
 * @author Frank Dellaert
 * @date December 5, 2014
 */

#include <gtsam/geometry/EssentialMatrix.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
EssentialMatrix EssentialMatrix::FromPose3(const Pose3& _1P2_,
    OptionalJacobian<5, 6> H) {
  const Rot3& _1R2_ = _1P2_.rotation();
  const Point3& _1T2_ = _1P2_.translation();
  if (!H) {
    // just make a direction out of translation and create E
    Unit3 direction(_1T2_);
    return EssentialMatrix(_1R2_, direction);
  } else {
    // Calculate the 5*6 Jacobian H = D_E_1P2
    // D_E_1P2 = [D_E_1R2 D_E_1T2], 5*3 wrpt rotation, 5*3 wrpt translation
    // First get 2*3 derivative from Unit3::FromPoint3
    Matrix23 D_direction_1T2;
    Unit3 direction = Unit3::FromPoint3(_1T2_, D_direction_1T2);
    *H << I_3x3, Z_3x3, //
    Matrix23::Zero(), D_direction_1T2 * _1R2_.matrix();
    return EssentialMatrix(_1R2_, direction);
  }
}

/* ************************************************************************* */
void EssentialMatrix::print(const string& s) const {
  cout << s;
  aRb_.print("R:\n");
  aTb_.print("d: ");
}

/* ************************************************************************* */
EssentialMatrix EssentialMatrix::retract(const Vector& xi) const {
  assert(xi.size() == 5);
  Vector3 omega(sub(xi, 0, 3));
  Vector2 z(sub(xi, 3, 5));
  Rot3 R = aRb_.retract(omega);
  Unit3 t = aTb_.retract(z);
  return EssentialMatrix(R, t);
}

/* ************************************************************************* */
Vector5 EssentialMatrix::localCoordinates(const EssentialMatrix& other) const {
  Vector5 v;
  v << aRb_.localCoordinates(other.aRb_),
      aTb_.localCoordinates(other.aTb_);
  return v;
}

/* ************************************************************************* */
Point3 EssentialMatrix::transform_to(const Point3& p, OptionalJacobian<3, 5> DE,
    OptionalJacobian<3, 3> Dpoint) const {
  Pose3 pose(aRb_, aTb_.point3());
  Matrix36 DE_;
  Point3 q = pose.transform_to(p, DE ? &DE_ : 0, Dpoint);
  if (DE) {
    // DE returned by pose.transform_to is 3*6, but we need it to be 3*5
    // The last 3 columns are derivative with respect to change in translation
    // The derivative of translation with respect to a 2D sphere delta is 3*2 aTb_.basis()
    // Duy made an educated guess that this needs to be rotated to the local frame
    Matrix35 H;
    H << DE_.block < 3, 3 > (0, 0), -aRb_.transpose() * aTb_.basis();
    *DE = H;
  }
  return q;
}

/* ************************************************************************* */
EssentialMatrix EssentialMatrix::rotate(const Rot3& cRb,
    OptionalJacobian<5, 5> HE, OptionalJacobian<5, 3> HR) const {

  // The rotation must be conjugated to act in the camera frame
  Rot3 c1Rc2 = aRb_.conjugate(cRb);

  if (!HE && !HR) {
    // Rotate translation direction and return
    Unit3 c1Tc2 = cRb * aTb_;
    return EssentialMatrix(c1Rc2, c1Tc2);
  } else {
    // Calculate derivatives
    Matrix23 D_c1Tc2_cRb; // 2*3
    Matrix2 D_c1Tc2_aTb; // 2*2
    Unit3 c1Tc2 = cRb.rotate(aTb_, D_c1Tc2_cRb, D_c1Tc2_aTb);
    if (HE)
      *HE << cRb.matrix(), Matrix32::Zero(), //
      Matrix23::Zero(), D_c1Tc2_aTb;
    if (HR) {
      throw runtime_error(
          "EssentialMatrix::rotate: derivative HR not implemented yet");
      /*
       HR->block<3, 3>(0, 0) << zeros(3, 3); // a change in the rotation yields ?
       HR->block<2, 3>(3, 0) << zeros(2, 3); // (2*3) * (3*3) ?
       */
    }
    return EssentialMatrix(c1Rc2, c1Tc2);
  }
}

/* ************************************************************************* */
double EssentialMatrix::error(const Vector3& vA, const Vector3& vB, //
    OptionalJacobian<1, 5> H) const {
  if (H) {
    // See math.lyx
    Matrix13 HR = vA.transpose() * E_ * skewSymmetric(-vB);
    Matrix12 HD = vA.transpose() * skewSymmetric(-aRb_.matrix() * vB)
        * aTb_.basis();
    *H << HR, HD;
  }
  return dot(vA, E_ * vB);
}

/* ************************************************************************* */
ostream& operator <<(ostream& os, const EssentialMatrix& E) {
  Rot3 R = E.rotation();
  Unit3 d = E.direction();
  os.precision(10);
  os << R.xyz().transpose() << " " << d.point3().vector().transpose() << " ";
  return os;
}

/* ************************************************************************* */
istream& operator >>(istream& is, EssentialMatrix& E) {
  double rx, ry, rz, dx, dy, dz;
  is >> rx >> ry >> rz; // Read the rotation rxyz
  is >> dx >> dy >> dz; // Read the translation dxyz

  // Create EssentialMatrix from rotation and translation
  Rot3 rot = Rot3::RzRyRx(rx, ry, rz);
  Unit3 dt = Unit3(dx, dy, dz);
  E = EssentialMatrix(rot, dt);

  return is;
}

/* ************************************************************************* */

} // gtsam

