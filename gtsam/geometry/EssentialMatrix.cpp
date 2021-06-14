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
EssentialMatrix EssentialMatrix::FromRotationAndDirection(const Rot3& aRb, const Unit3& aTb,
                                                          OptionalJacobian<5, 3> H1,
                                                          OptionalJacobian<5, 2> H2) {
  if (H1)
    *H1 << I_3x3, Matrix23::Zero();
  if (H2)
    *H2 << Matrix32::Zero(), I_2x2;
  return EssentialMatrix(aRb, aTb);
}

/* ************************************************************************* */
EssentialMatrix EssentialMatrix::FromPose3(const Pose3& aPb,
    OptionalJacobian<5, 6> H) {
  const Rot3& aRb = aPb.rotation();
  const Point3& aTb = aPb.translation();
  if (!H) {
    // just make a direction out of translation and create E
    Unit3 direction(aTb);
    return EssentialMatrix(aRb, direction);
  } else {
    // Calculate the 5*6 Jacobian H = D_E_1P2
    // D_E_1P2 = [D_E_1R2 D_E_1T2], 5*3 wrpt rotation, 5*3 wrpt translation
    // First get 2*3 derivative from Unit3::FromPoint3
    Matrix23 D_direction_1T2;
    Unit3 direction = Unit3::FromPoint3(aTb, D_direction_1T2);
    *H << I_3x3, Z_3x3, //
    Matrix23::Zero(), D_direction_1T2 * aRb.matrix();
    return EssentialMatrix(aRb, direction);
  }
}

/* ************************************************************************* */
void EssentialMatrix::print(const string& s) const {
  cout << s;
  rotation().print("R:\n");
  direction().print("d: ");
}

/* ************************************************************************* */
Point3 EssentialMatrix::transformTo(const Point3& p, OptionalJacobian<3, 5> DE,
    OptionalJacobian<3, 3> Dpoint) const {
  Pose3 pose(rotation(), direction().point3());
  Matrix36 DE_;
  Point3 q = pose.transformTo(p, DE ? &DE_ : 0, Dpoint);
  if (DE) {
    // DE returned by pose.transformTo is 3*6, but we need it to be 3*5
    // The last 3 columns are derivative with respect to change in translation
    // The derivative of translation with respect to a 2D sphere delta is 3*2 direction().basis()
    // Duy made an educated guess that this needs to be rotated to the local frame
    Matrix35 H;
    H << DE_.block < 3, 3 > (0, 0), -rotation().transpose() * direction().basis();
    *DE = H;
  }
  return q;
}

/* ************************************************************************* */
EssentialMatrix EssentialMatrix::rotate(const Rot3& cRb,
    OptionalJacobian<5, 5> HE, OptionalJacobian<5, 3> HR) const {

  // The rotation must be conjugated to act in the camera frame
  Rot3 c1Rc2 = rotation().conjugate(cRb);

  if (!HE && !HR) {
    // Rotate translation direction and return
    Unit3 c1Tc2 = cRb * direction();
    return EssentialMatrix(c1Rc2, c1Tc2);
  } else {
    // Calculate derivatives
    Matrix23 D_c1Tc2_cRb; // 2*3
    Matrix2 D_c1Tc2_aTb; // 2*2
    Unit3 c1Tc2 = cRb.rotate(direction(), D_c1Tc2_cRb, D_c1Tc2_aTb);
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
    Matrix12 HD = vA.transpose() * skewSymmetric(-rotation().matrix() * vB)
        * direction().basis();
    *H << HR, HD;
  }
  return dot(vA, E_ * vB);
}

/* ************************************************************************* */
ostream& operator <<(ostream& os, const EssentialMatrix& E) {
  Rot3 R = E.rotation();
  Unit3 d = E.direction();
  os.precision(10);
  os << R.xyz().transpose() << " " << d.point3().transpose() << " ";
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

