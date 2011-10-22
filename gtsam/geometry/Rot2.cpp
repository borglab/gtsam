/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2.cpp
 * @date Dec 9, 2009
 * @author Frank Dellaert
 * @brief 2D Rotations
 */

#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;

namespace gtsam {

/** Explicit instantiation of base class to export members */
INSTANTIATE_LIE(Rot2);

/* ************************************************************************* */
Rot2 Rot2::fromCosSin(double c, double s) {
	if (fabs(c * c + s * s - 1.0) > 1e-9) {
		double norm_cs = sqrt(c*c + s*s);
		c = c/norm_cs;
		s = s/norm_cs;
	}
	return Rot2(c, s);
}

/* ************************************************************************* */
Rot2 Rot2::atan2(double y, double x) {
	Rot2 R(x, y);
	return R.normalize();
}

/* ************************************************************************* */
void Rot2::print(const string& s) const {
	cout << s << ": " << theta() << endl;
}

/* ************************************************************************* */
bool Rot2::equals(const Rot2& R, double tol) const {
	return fabs(c_ - R.c_) <= tol && fabs(s_ - R.s_) <= tol;
}

/* ************************************************************************* */
Rot2& Rot2::normalize() {
	double scale = c_*c_ + s_*s_;
	if(fabs(scale-1.0)>1e-10) {
		scale = pow(scale, -0.5);
		c_ *= scale;
		s_ *= scale;
	}
	return *this;
}

/* ************************************************************************* */
Matrix Rot2::matrix() const {
	return Matrix_(2, 2, c_, -s_, s_, c_);
}

/* ************************************************************************* */
Matrix Rot2::transpose() const {
	return Matrix_(2, 2, c_, s_, -s_, c_);
}

/* ************************************************************************* */
// see doc/math.lyx, SO(2) section
Point2 Rot2::rotate(const Point2& p, boost::optional<Matrix&> H1,
		boost::optional<Matrix&> H2) const {
	const Point2 q = Point2(c_ * p.x() + -s_ * p.y(), s_ * p.x() + c_ * p.y());
	if (H1) *H1 = Matrix_(2, 1, -q.y(), q.x());
	if (H2) *H2 = matrix();
	return q;
}

/* ************************************************************************* */
// see doc/math.lyx, SO(2) section
Point2 Rot2::unrotate(const Point2& p,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
	if (H1) *H1 = Matrix_(2, 1, q.y(), -q.x());  // R_{pi/2}q
	if (H2) *H2 = transpose();
	return q;
}

/* ************************************************************************* */
Rot2 Rot2::relativeBearing(const Point2& d, boost::optional<Matrix&> H) {
	double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
	if(fabs(n) > 1e-5) {
	  if (H) *H = Matrix_(1, 2, -y / d2, x / d2);
	  return Rot2::fromCosSin(x / n, y / n);
	} else {
	  if (H) *H = Matrix_(1,2, 0.0, 0.0);
	  return Rot2();
	}
}

/* ************************************************************************* */

} // gtsam
