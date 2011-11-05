/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Point3.cpp
 * @brief 3D Point
 */

#include <cmath>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Lie-inl.h>

namespace gtsam {

/** Explicit instantiation of base class to export members */
INSTANTIATE_LIE(Point3);

/* ************************************************************************* */
bool Point3::equals(const Point3 & q, double tol) const {
	return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol && fabs(z_ - q.z()) < tol);
}

/* ************************************************************************* */

void Point3::print(const std::string& s) const {
	std::cout << s << "(" << x_ << ", " << y_ <<  ", " << z_ << ")" << std::endl;
}

/* ************************************************************************* */

bool Point3::operator== (const Point3& q) const {
	return x_ == q.x_ && y_ == q.y_ && z_ == q.z_;
}

/* ************************************************************************* */
Point3 Point3::operator+(const Point3& q) const {
	return Point3( x_ + q.x_, y_ + q.y_, z_ + q.z_ );
}

/* ************************************************************************* */
Point3 Point3::operator- (const Point3& q) const {
	return Point3( x_ - q.x_, y_ - q.y_, z_ - q.z_ );
}
/* ************************************************************************* */
Point3 Point3::operator*(double s) const {
	return Point3(x_ * s, y_ * s, z_ * s);
}
/* ************************************************************************* */
Point3 Point3::operator/(double s) const {
	return Point3(x_ / s, y_ / s, z_ / s);
}
/* ************************************************************************* */
Point3 Point3::add(const Point3 &q,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	if (H1) *H1 = eye(3,3);
	if (H2) *H2 = eye(3,3);
	return *this + q;
}
/* ************************************************************************* */
Point3 Point3::sub(const Point3 &q,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	if (H1) *H1 = eye(3,3);
	if (H2) *H2 = -eye(3,3);
	return *this - q;
}
/* ************************************************************************* */
Point3 Point3::cross(const Point3 &q) const {
	return Point3( y_*q.z_ - z_*q.y_,
			z_*q.x_ - x_*q.z_,
			x_*q.y_ - y_*q.x_ );
}
/* ************************************************************************* */
double Point3::dot(const Point3 &q) const {
	return ( x_*q.x_ + y_*q.y_ + z_*q.z_ );
}
/* ************************************************************************* */
double Point3::norm() const {
	return sqrt( x_*x_ + y_*y_ + z_*z_ );
}
/* ************************************************************************* */

} // namespace gtsam
