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
	bool assert_equal(const Point2& p, const Point2& q, double tol) {
		if (fabs(p.x() - q.x()) < tol && fabs(p.y() - q.y()) < tol) return true;
		printf("not equal:\n");
		p.print("p = ");
		q.print("q = ");
		(p - q).print("p-q = ");
		return false;
	}

/* ************************************************************************* */

} // namespace gtsam
