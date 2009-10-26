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
	/** faster than below ?                                                      */
	/* ************************************************************************* */
	Rot3 rodriguez(const Vector& w, double t) {
		double l_w = 0.0;
		for (int i = 0; i < 3; i++)
			l_w += pow(w(i), 2.0);
		if (l_w != 1.0) throw domain_error("rodriguez: length of w should be 1");

		double ct = cos(t), st = sin(t);

		Point3 r1 = Point3(ct + w(0) * w(0) * (1 - ct), w(2) * st + w(0) * w(1) * (1 - ct), -w(1) * st + w(0) * w(2) * (1 - ct));
		Point3 r2 = Point3(w(1) * w(0) * (1 - ct) - w(2) * st, w(1) * w(1) * (1 - ct) + ct, w(1) * w(2) * (1 - ct) + w(0) * st);
		Point3 r3 = Point3(w(1) * st + w(2) * w(0) * (1 - ct), -w(0) * st + w(2) * w(1) * (1 - ct), ct + w(2) * w(2) * (1 - ct));

		return Rot3(r1, r2, r3);
	}

	/* ************************************************************************* */
	Rot3 rodriguez(double wx, double wy, double wz) {
		Matrix J = skewSymmetric(wx, wy, wz);
		double t2 = wx * wx + wy * wy + wz * wz;
		if (t2 < 1e-10) return Rot3();
		double t = sqrt(t2);
		Matrix R = eye(3, 3) + sin(t) / t * J + (1.0 - cos(t)) / t2 * (J * J);
		return R; // matrix constructor will be tripped
	}

	/* ************************************************************************* */
	Rot3 rodriguez(const Vector& v) {
		return rodriguez(v(0), v(1), v(2));
	}

	/* ************************************************************************* */
	Rot3 exmap(const Rot3& R, const Vector& v) {
		return rodriguez(v) * R;
	}

	/* ************************************************************************* */
	Rot3 Rot3::exmap(const Vector& v) const {
		if (zero(v)) return (*this);
		return rodriguez(v) * (*this);
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
