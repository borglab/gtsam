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
	Vector Rot3::log() const {
		double tr = r1_.x()+r2_.y()+r3_.z();
		if (tr==3.0) return ones(3);
		if (tr==-1.0) throw domain_error("Rot3::log: trace == -1 not yet handled :-(");;
		double theta = acos((tr-1.0)/2.0);
		return (theta/2.0/sin(theta))*Vector_(3,
				r2_.z()-r3_.y(),
				r3_.x()-r1_.z(),
				r1_.y()-r2_.x());
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
	Point3 Rot3::unrotate(const Point3& p) const {
		return Point3(
				r1_.x() * p.x() + r1_.y() * p.y() + r1_.z() * p.z(),
				r2_.x() * p.x() + r2_.y() * p.y() + r2_.z() * p.z(),
				r3_.x() * p.x() + r3_.y() * p.y() + r3_.z() * p.z()
				);
	}

	/* ************************************************************************* */
  Vector Rot3::ypr() const {
  	Matrix I;Vector q;
  	boost::tie(I,q)=RQ(matrix());
  	return q;
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
  Vector log(const Rot3& R, const Rot3& S) {
  	return between(R,S).log();
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
	Rot3 between(const Rot3& R1, const Rot3& R2) {
		return R2 * R1.inverse();
	}

	/* ************************************************************************* */
	pair<Matrix,Vector> RQ(const Matrix& A) {
		double A21 = A(2, 1), A22 = A(2, 2), a = sqrt(A21 * A21 + A22 * A22);
		double Cx =  A22 / a; //cosX
		double Sx = -A21 / a; //sinX
		Matrix Qx = Matrix_(3, 3,
				1.0, 0.0, 0.0,
				0.0,  Cx, -Sx,
				0.0,  Sx, Cx);
		Matrix B = A * Qx;

		double B20 = B(2, 0), B22 = B(2, 2), b = sqrt(B20 * B20 + B22 * B22);
		double Cy = B22 / b; //cosY
		double Sy = B20 / b; //sinY
		Matrix Qy = Matrix_(3,3,
				 Cy, 0.0,  Sy,
				0.0, 1.0, 0.0,
				-Sy, 0.0,  Cy);
		Matrix C = B * Qy;

		double C10 = C(1, 0), C11 = C(1, 1), c = sqrt(C10 * C10 + C11 * C11);
		double Cz =  C11 / c; //cosZ
		double Sz = -C10 / c; //sinZ
		Matrix Qz = Matrix_(3, 3,
				 Cz, -Sz, 0.0,
				 Sz,  Cz, 0.0,
				0.0, 0.0, 1.0);
		Matrix R = C * Qz;

		Vector angles(3);
		angles(0) = -atan2(Sx, Cx);
		angles(1) = -atan2(Sy, Cy);
		angles(2) = -atan2(Sz, Cz);

		return make_pair(R,angles);
	}

/* ************************************************************************* */

} // namespace gtsam
