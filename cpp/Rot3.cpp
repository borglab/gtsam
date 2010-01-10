/**
 * @file    Rot3.cpp
 * @brief   Rotation (internal: 3*3 matrix representation*)
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

#include "Rot3.h"
#include "Lie-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  template class Lie<Rot3>;

  /* ************************************************************************* */
	// static member functions to construct rotations

  Rot3 Rot3::Rx(double t) {
  	double st = sin(t), ct = cos(t);
  	return Rot3(
  			1,  0,  0,
  			0, ct,-st,
  			0, st, ct);
  }

  Rot3 Rot3::Ry(double t) {
  	double st = sin(t), ct = cos(t);
  	return Rot3(
  			 ct, 0, st,
  			  0, 1,  0,
  			-st, 0, ct);
  }

  Rot3 Rot3::Rz(double t) {
  	double st = sin(t), ct = cos(t);
  	return Rot3(
  			ct,-st, 0,
  			st, ct, 0,
  			 0,  0, 1);
  }

  // Considerably faster than composing matrices above !
  Rot3 Rot3::RzRyRx(double x, double y, double z) {
  	double cx=cos(x),sx=sin(x);
  	double cy=cos(y),sy=sin(y);
  	double cz=cos(z),sz=sin(z);
  	double ss_ = sx * sy;
  	double cs_ = cx * sy;
  	double sc_ = sx * cy;
  	double cc_ = cx * cy;
  	double c_s = cx * sz;
  	double s_s = sx * sz;
		double _cs = cy * sz;
  	double _cc = cy * cz;
		double s_c = sx * cz;
		double c_c = cx * cz;
		double ssc = ss_ * cz, csc = cs_ * cz, sss = ss_ * sz, css = cs_ * sz;
  	return Rot3(
  			_cc,- c_s + ssc,  s_s + csc,
  			_cs,  c_c + sss, -s_c + css,
				-sy,        sc_,        cc_
  			);
  }

  /* ************************************************************************* */
  bool Rot3::equals(const Rot3 & R, double tol) const {
    return equal_with_abs_tol(matrix(), R.matrix(), tol);
  }

  /* ************************************************************************* */
  Matrix Rot3::matrix() const {
    double r[] = { r1_.x(), r2_.x(), r3_.x(),
        r1_.y(), r2_.y(), r3_.y(),
        r1_.z(), r2_.z(), r3_.z() };
    return Matrix_(3,3, r);
  }

  /* ************************************************************************* */
  Matrix Rot3::transpose() const {
    double r[] = { r1_.x(), r1_.y(), r1_.z(),
        r2_.x(), r2_.y(), r2_.z(),
        r3_.x(), r3_.y(), r3_.z()};
    return Matrix_(3,3, r);
  }

  /* ************************************************************************* */
  Point3 Rot3::column(int index) const{
    if(index == 3)
      return r3_;
    else if (index == 2)
      return r2_;
    else
      return r1_; // default returns r1
  }

  /* ************************************************************************* */
  Vector Rot3::xyz() const {
    Matrix I;Vector q;
    boost::tie(I,q)=RQ(matrix());
    return q;
  }

  Vector Rot3::ypr() const {
  	Vector q = xyz();
    return Vector_(3,q(2),q(1),q(0));
  }

  /* ************************************************************************* */
  Rot3 rodriguez(const Vector& n, double t) {
    double n0 = n(0), n1=n(1), n2=n(2);
    double n00 = n0*n0, n11 = n1*n1, n22 = n2*n2;
#ifndef NDEBUG
    double l_n = n00+n11+n22;
    if (fabs(l_n-1.0)>1e-9) throw domain_error("rodriguez: length of n should be 1");
#endif

    double ct = cos(t), st = sin(t), ct_1 = 1 - ct;

    double s0 = n0 * st, s1 = n1 * st, s2 = n2 * st;
    double C01 = ct_1*n0*n1, C02 = ct_1*n0*n2, C12 = ct_1*n1*n2;
    double C00 = ct_1*n00, C11 = ct_1*n11, C22 = ct_1*n22;

    Point3 r1 = Point3( ct + C00,  s2 + C01, -s1 + C02);
    Point3 r2 = Point3(-s2 + C01,  ct + C11,  s0 + C12);
    Point3 r3 = Point3( s1 + C02, -s0 + C12,  ct + C22);

    return Rot3(r1, r2, r3);
  }

  /* ************************************************************************* */
  Rot3 rodriguez(const Vector& w) {
    double t = norm_2(w);
    if (t < 1e-5) return Rot3();
    return rodriguez(w/t, t);
  }

  /* ************************************************************************* */
  Point3 rotate(const Rot3& R, const Point3& p) {
    return R.r1() * p.x() + R.r2() * p.y() + R.r3() * p.z();
  }

  /* ************************************************************************* */
  Matrix Drotate1(const Rot3& R, const Point3& p) {
    Point3 q = R * p;
    return skewSymmetric(-q.x(), -q.y(), -q.z());
  }

  /* ************************************************************************* */
  Matrix Drotate2(const Rot3& R) {
    return R.matrix();
  }

  /* ************************************************************************* */
  Point3 unrotate(const Rot3& R, const Point3& p) {
    return Point3(
        R.r1().x() * p.x() + R.r1().y() * p.y() + R.r1().z() * p.z(),
        R.r2().x() * p.x() + R.r2().y() * p.y() + R.r2().z() * p.z(),
        R.r3().x() * p.x() + R.r3().y() * p.y() + R.r3().z() * p.z()
    );
  }

  /* ************************************************************************* */
  /** see libraries/caml/geometry/math.lyx, derivative of unrotate              */
  /* ************************************************************************* */
  Matrix Dunrotate1(const Rot3 & R, const Point3 & p) {
    Point3 q = unrotate(R,p);
    return skewSymmetric(q.x(), q.y(), q.z()) * R.transpose();
  }

  /* ************************************************************************* */
  Matrix Dunrotate2(const Rot3 & R) {
    return R.transpose();
  }

  /* ************************************************************************* */
  Matrix Dcompose1(const Rot3& R1, const Rot3& R2){
  	return eye(3);
  }

  /* ************************************************************************* */
  Matrix Dcompose2(const Rot3& R1, const Rot3& R2){
    return R1.matrix();
  }

  /* ************************************************************************* */
  Matrix Dbetween1(const Rot3& R1, const Rot3& R2){
  	return -between(R1,R2).matrix();
  }

  /* ************************************************************************* */
  Matrix Dbetween2(const Rot3& R1, const Rot3& R2){
    return eye(3);
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> RQ(const Matrix& A) {

		double x = -atan2(-A(2, 1), A(2, 2));
		Rot3 Qx = Rot3::Rx(-x);
		Matrix B = A * Qx.matrix();

		double y = -atan2(B(2, 0), B(2, 2));
		Rot3 Qy = Rot3::Ry(-y);
		Matrix C = B * Qy.matrix();

		double z = -atan2(-C(1, 0), C(1, 1));
		Rot3 Qz = Rot3::Rz(-z);
		Matrix R = C * Qz.matrix();

		Vector xyz = Vector_(3, x, y, z);
		return make_pair(R, xyz);
	}

  /* ************************************************************************* */

} // namespace gtsam
