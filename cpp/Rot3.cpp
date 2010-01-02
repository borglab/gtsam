/**
 * @file    Rot3.cpp
 * @brief   Rotation (internal: 3*3 matrix representation*)
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

#include "Rot3.h"

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
	bool Rot3::equals(const Rot3 & R, double tol) const {
		return equal_with_abs_tol(matrix(), R.matrix(), tol);
	}

	/* ************************************************************************* */
	Rot3 Rot3::exmap(const Vector& v) const {
		if (zero(v)) return (*this);
		return rodriguez(v) * (*this);
	}

  /* ************************************************************************* */
  Vector Rot3::vector() const {
    double r[] = { r1_.x(), r1_.y(), r1_.z(),
	     r2_.x(), r2_.y(), r2_.z(),
	     r3_.x(), r3_.y(), r3_.z() };
    Vector v(9);
    copy(r,r+9,v.begin());
    return v;
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
  Rot3 Rot3::inverse() const {
  	return Rot3(
  			r1_.x(), r1_.y(), r1_.z(),
  			r2_.x(), r2_.y(), r2_.z(),
  			r3_.x(), r3_.y(), r3_.z());
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
	Rot3 exmap(const Rot3& R, const Vector& v) {
		return R.exmap(v);
	}

	/* ************************************************************************* */
	Point3 rotate(const Rot3& R, const Point3& p) {
		return R * p;
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
		return R.unrotate(p);
	}

	/* ************************************************************************* */
	/** see libraries/caml/geometry/math.lyx, derivative of unrotate              */
	/* ************************************************************************* */
	Matrix Dunrotate1(const Rot3 & R, const Point3 & p) {
		Point3 q = R.unrotate(p);
		return skewSymmetric(q.x(), q.y(), q.z()) * R.transpose();
	}

	/* ************************************************************************* */
	Matrix Dunrotate2(const Rot3 & R) {
		return R.transpose();
	}

	/* ************************************************************************* */
	/** This function receives a rotation 3 by 3 matrix and returns 3 rotation angles.
	 *  The implementation is based on the algorithm in multiple view geometry
	 *  the function returns a vector that its arguments are: thetax, thetay, thetaz in radians.
	 */
	/* ************************************************************************* */
	Vector RQ(Matrix R) {
		double Cx = R(2, 2) / (double) ((sqrt(pow((double) (R(2, 2)), 2.0) + pow(
				(double) (R(2, 1)), 2.0)))); //cosX
		double Sx = -R(2, 1) / (double) ((sqrt(pow((double) (R(2, 2)), 2.0) + pow(
				(double) (R(2, 1)), 2.0)))); //sinX
		Matrix Qx(3, 3);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Qx(i, j) = 0;

		Qx(0, 0) = 1;
		Qx(1, 1) = Cx;
		Qx(1, 2) = -Sx;
		Qx(2, 1) = Sx;
		Qx(2, 2) = Cx;
		R = R * Qx;
		double Cy = R(2, 2) / (sqrt(pow((double) (R(2, 2)), 2.0) + pow((double) (R(
				2, 0)), 2.0))); //cosY
		double Sy = R(2, 0) / (sqrt(pow((double) (R(2, 2)), 2.0) + pow((double) (R(
				2, 0)), 2.0))); //sinY
		Matrix Qy(3, 3);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Qy(i, j) = 0;

		Qy(0, 0) = Cy;
		Qy(0, 2) = Sy;
		Qy(1, 1) = 1;
		Qy(2, 0) = -Sy;
		Qy(2, 2) = Cy;
		R = R * Qy;
		double Cz = R(1, 1) / (sqrt(pow((double) (R(1, 1)), 2.0) + pow((double) (R(
				1, 0)), 2.0))); //cosZ
		double Sz = -R(1, 0) / (sqrt(pow((double) (R(1, 1)), 2.0) + pow(
				(double) (R(1, 0)), 2.0)));//sinZ
		Matrix Qz(3, 3);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Qz(i, j) = 0;
		Qz(0, 0) = Cz;
		Qz(0, 1) = -Sz;
		Qz(1, 0) = Sz;
		Qz(1, 1) = Cz;
		Qz(2, 2) = 1;
		R = R * Qz;
		double pi = atan2(sqrt(2.0) / 2.0, sqrt(2.0) / 2.0) * 4.0;

		Vector result(3);
		result(0) = -atan2(Sx, Cx);
		result(1) = -atan2(Sy, Cy);
		result(2) = -atan2(Sz, Cz);

		return result;
	}

/* ************************************************************************* */

} // namespace gtsam
