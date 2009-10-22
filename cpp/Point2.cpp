/**
 * @file    Point2.cpp
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#include "Point2.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	void Point2::print(const string& s) const {
		cout << s << "(" << x_ << ", " << y_ << ")" << endl;
	}

	/* ************************************************************************* */
	bool Point2::equals(const Point2& q, double tol) const {
		return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
	}

	/* ************************************************************************* */
	double Point2::dist(const Point2& p2) const {
		return sqrt(pow(x() - p2.x(), 2.0) + pow(y() - p2.y(), 2.0));
	}

/* ************************************************************************* */

} // namespace gtsam
