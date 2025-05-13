/**
 * @file    homography.h
 * @brief   Homography utilities
 * @author  Alireza Fathi
 */

#include "homography.h"

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
Matrix getL(const Matrix& m /*4by2*/,
            const Matrix& M /*4by3*/) /*returns a 8by9*/
{
  Matrix L = zeros(8, 9);

  for (int i = 0, k = 0; i < 8; i += 2, k++) {
    const double xi = m(k, 0), yi = m(k, 1);
    for (int j = 0; j < 3; j++) {
      const double Mkj = M(k, j);
      L(i + 1, j + 3) = Mkj;
      L(i + 1, j + 6) = -yi * Mkj;
      L(i, j) = Mkj;
      L(i, j + 6) = -xi * Mkj;
    }
  }
  return L;
}

/* ************************************************************************* */
// local routine that calculates homography with SVD in place (destroying L)
Matrix getH_inplace(Matrix& L) {
  // perform SVD on 8*9 matrix
  Matrix V;
  Vector s;
  svd(L, s, V);

  // find smallest singular values
  int j0 = 0;
  for (int j = 0; j < 9; j++)
    if (s(j) < s(j0)) j0 = j;

  // copy j0 column corresponding to smallest singular value into H, row order
  Matrix H(3, 3);
  for (int i = 0, k = 0; i < 3; i++)
    for (int j = 0; j < 3; j++, k++) H(i, j) = V(k, j0);

  return H;
}

/* ************************************************************************* */
// local routine that calculates homography with SVD in place (destroying L)
Matrix normalized_getH_inplace(Matrix& L) {
  // print(L);
  Vector coefs(9);
  double mw = L(0, 0);
  double mh = L(0, 1);       // markerWidth and the markerHeight
  double uc = L(0, 6) / mw;  // pixels are about this size
  coefs(0) = mw;
  coefs(1) = mh;
  coefs(2) = 1.0;
  coefs(3) = mw;
  coefs(4) = mh;
  coefs(5) = 1.0;
  coefs(6) = mw * uc;
  coefs(7) = mh * uc;
  coefs(8) = uc;

  // print(coefs);

  // normalizing L
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 9; j++) L(i, j) = L(i, j) / (coefs(j));

  // print(L);

  // perform SVD on 8*9 matrix
  Matrix V;
  Vector s;
  svd(L, s, V);

  // find smallest singular values
  int j0 = 0;
  for (int j = 0; j < 9; j++)
    if (s(j) < s(j0)) j0 = j;

  // copy j0 column corresponding to smallest singular value into H, row order
  Matrix H(3, 3);
  for (int i = 0, k = 0; i < 3; i++)
    for (int j = 0; j < 3; j++, k++) H(i, j) = V(k, j0) / (coefs(k));

  return H;
}

/* ************************************************************************* */
// exposed version that does not change L at the expense of a copy
Matrix getH(const Matrix& L) {
  Matrix Lcopy = L;  // copy L
  return getH_inplace(Lcopy);
}

/* ************************************************************************* */
// version that works from the matrices directly
Matrix getH(const Matrix& m /*4by2*/, const Matrix& M /*4by3*/) {
  Matrix L = getL(m, M);   // build L
  return getH_inplace(L);  // compute H, destroying L in process
}

/* ************************************************************************* */
// see papers/EasySLAM/conference.lyx
Pose3 getTransformationFromMarkerToCamera(const Matrix& H, const Matrix& K) {
  // get homography columns
  Vector h1(3), h2(3), h3(3);
  for (int i = 0; i < 3; i++) {
    h1(i) = H(i, 0);
    h2(i) = H(i, 1);
    h3(i) = H(i, 2);
  }
  // take out known calibration
  Matrix invK = inverse(K);
  Vector v1 = invK * h1;
  Vector v2 = invK * h2;
  Vector v3 = invK * h3;

  double lambda1 = norm_2(v1), lambda2 = norm_2(v2);
  //::print(H);
  // printf("%f %f\n", lambda1, lambda2);
  if (v3(2) < 0) {
    lambda1 = -lambda1;
    lambda2 = -lambda2;
  }
  double lambda3 = (lambda1 + lambda2) / 2.0;  // average

  Point3 r1 = Point3(v1 / lambda3);
  Point3 r2 = Point3(v2 / lambda3);
  Point3 t = Point3(v3 / lambda3);
  Point3 r3 = cross(r1, r2);

  Rot3 R(r1, r2, r3);

  Rot3 fixedR = fixToRotation_zhang(R);
  // Rot3 fixedR = R;

  return Pose3(fixedR, t);
}
/* ************************************************************************* */
Rot3 fixToRotation_zhang(const Rot3& Q) {
  Matrix Qm = Q.matrix();
  Vector S;
  Matrix U, V;
  svd(Qm, U, S, V);
  // Q.print("Q");
  Matrix VT(
      V);  // transpose V!, isn't there a transpose function in Matrix class?
  //::print(U,"U");
  //::print(VT,"V");
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) VT(i, j) = V(j, i);
  Rot3 res = Rot3(U * VT);
  // res.print("res");
  return res;
}
