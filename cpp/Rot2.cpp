/*
 * Rot2.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#include "Rot2.h"
#include "Lie-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Rot2);

	/* ************************************************************************* */
	void Rot2::print(const string& s) const {
		cout << s << ":" << theta() << endl;
	}

	/* ************************************************************************* */
	bool Rot2::equals(const Rot2& R, double tol) const {
		return fabs(c_ - R.c_) <= tol && fabs(s_ - R.s_) <= tol;
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
	Matrix Rot2::negtranspose() const {
		return Matrix_(2, 2, -c_, -s_, s_, -c_);
	}

	/* ************************************************************************* */
	Point2 Rot2::rotate(const Point2& p) const {
		return Point2(c_ * p.x() + -s_ * p.y(), s_ * p.x() + c_ * p.y());
	}

	/* ************************************************************************* */
	Point2 Rot2::unrotate(const Point2& p) const {
		return Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
	}

	/* ************************************************************************* */
	// see libraries/caml/geometry/math.ml
	Point2 rotate(const Rot2 & R, const Point2& p,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		Point2 q = R * p;
		if (H1) *H1 = Matrix_(2, 1, -q.y(), q.x());
		if (H2) *H2 = R.matrix();
		return q;
	}

	/* ************************************************************************* */
	// see libraries/caml/geometry/math.lyx, derivative of unrotate
	Point2 unrotate(const Rot2 & R, const Point2& p,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		Point2 q = R.unrotate(p);
		if (H1) *H1 = Matrix_(2, 1, q.y(), -q.x());
		if (H2) *H2 = R.transpose();
		return q;
	}

	/* ************************************************************************* */
	Rot2 relativeBearing(const Point2& d) {
		double n = d.norm();
		return Rot2(d.x() / n, d.y() / n);
	}

	/* ************************************************************************* */
	Rot2 relativeBearing(const Point2& d, boost::optional<Matrix&> H) {
		double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
		if (H) *H = Matrix_(1, 2, -y / d2, x / d2);
		return Rot2(x / n, y / n);
	}

/* ************************************************************************* */

} // gtsam
