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
  INSTANTIATE_LIE(Rot3);

	static const Matrix I3 = eye(3);

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
  // Log map at identity - return the canonical coordinates of this rotation
  inline Vector logmap(const Rot3& R) {
    double tr = R.r1().x()+R.r2().y()+R.r3().z();
    if (fabs(tr-3.0) < 1e-10)   // when theta = 0, +-2pi, +-4pi, etc.
      return zero(3);
    else if (tr==-1.0) { // when theta = +-pi, +-3pi, +-5pi, etc.
      if(R.r3().z() != -1.0)
        return (boost::math::constants::pi<double>() / sqrt(2.0+2.0*R.r3().z())) *
        Vector_(3, R.r3().x(), R.r3().y(), 1.0+R.r3().z());
      else if(R.r2().y() != -1.0)
        return (boost::math::constants::pi<double>() / sqrt(2.0+2.0*R.r2().y())) *
        Vector_(3, R.r2().x(), 1.0+R.r2().y(), R.r2().z());
      else if(R.r1().x() != -1.0)
        return (boost::math::constants::pi<double>() / sqrt(2.0+2.0*R.r1().x())) *
        Vector_(3, 1.0+R.r1().x(), R.r1().y(), R.r1().z());
    } else {
      double theta = acos((tr-1.0)/2.0);
      return (theta/2.0/sin(theta))*Vector_(3,
          R.r2().z()-R.r3().y(),
          R.r3().x()-R.r1().z(),
          R.r1().y()-R.r2().x());
    }
  }

  /* ************************************************************************* */
  Rot3 rodriguez(const Vector& w, double theta) {
  	// get components of axis \omega
    double wx = w(0), wy=w(1), wz=w(2);
    double w2_xx = wx*wx, w2_yy = wy*wy, w2_zz = wz*wz;
#ifndef NDEBUG
    double l_n = w2_xx + w2_yy + w2_zz;
    if (fabs(l_n-1.0)>1e-9) throw domain_error("rodriguez: length of n should be 1");
#endif

    double c = cos(theta), s = sin(theta), c_1 = 1 - c;

    double swx = wx * s, swy = wy * s, swz = wz * s;
    double C00 = c_1*w2_xx, C01 = c_1*wx*wy, C02 = c_1*wx*wz;
    double                  C11 = c_1*w2_yy, C12 = c_1*wy*wz;
    double                                   C22 = c_1*w2_zz;

    // Important: these are columns, so this reads transposed !!!!
    Point3 r1 = Point3(   c + C00,  swz + C01, -swy + C02);
    Point3 r2 = Point3(-swz + C01,    c + C11,  swx + C12);
    Point3 r3 = Point3( swy + C02, -swx + C12,    c + C22);

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
    return R.matrix() * skewSymmetric(-p.x(), -p.y(), -p.z());
  }

  /* ************************************************************************* */
  Matrix Drotate2(const Rot3& R) {
    return R.matrix();
  }

  /* ************************************************************************* */
  Point3 unrotate(const Rot3& R, const Point3& p) {
    const Matrix Rt(R.transpose());
    return Rt*p.vector(); // q = Rt*p
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SO(3) section
  Point3 unrotate(const Rot3& R, const Point3& p,
  		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
    const Matrix Rt(R.transpose());
    Point3 q(Rt*p.vector()); // q = Rt*p
    if (H1) *H1 = skewSymmetric(q.x(), q.y(), q.z());
    if (H2) *H2 = Rt;
    return q;
  }

  /* ************************************************************************* */
  Matrix Dcompose1(const Rot3& R1, const Rot3& R2){
    return R2.transpose();
  }

  /* ************************************************************************* */
  Matrix Dcompose2(const Rot3& R1, const Rot3& R2){
  	return I3;
  }

  /* ************************************************************************* */
  Matrix Dbetween1(const Rot3& R1, const Rot3& R2){
  	return -(R2.transpose()*R1.matrix());
  }

  /* ************************************************************************* */
  Matrix Dbetween2(const Rot3& R1, const Rot3& R2){
  	return I3;
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
