/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point2.cpp
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;

namespace gtsam {

/** Explicit instantiation of base class to export members */
INSTANTIATE_LIE(Point2);

/* ************************************************************************* */
void Point2::print(const string& s) const {
	cout << s << "(" << x_ << ", " << y_ << ")" << endl;
}

/* ************************************************************************* */
bool Point2::equals(const Point2& q, double tol) const {
	return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
}

/* ************************************************************************* */
double Point2::norm() const {
	return sqrt(x_*x_ + y_*y_);
}

} // namespace gtsam
