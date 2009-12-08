/**
 * @file  Pose2.cpp
 * @brief 2D Pose
 */

#include "Pose2.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	void Pose2::print(const string& s) const {
		cout << s << "(" << x_ << ", " << y_ << ", " << theta_ << ")" << endl;
	}

	/* ************************************************************************* */
	bool Pose2::equals(const Pose2& q, double tol) const {
		return (fabs(x_ - q.x_) < tol && fabs(y_ - q.y_) < tol && fabs(theta_
				- q.theta_) < tol);
	}

	/* ************************************************************************* */
	Pose2 Pose2::exmap(const Vector& v) const {
		return Pose2(x_ + v(0), y_ + v(1), theta_ + v(2));
	}

	/* ************************************************************************* */
	Vector Pose2::vector() const {
		Vector v(3);
		v(0) = x_;
		v(1) = y_;
		v(2) = theta_;
		return v;
	}

	/* ************************************************************************* */
	Pose2 Pose2::rotate(double theta) const {
		double c = cos(theta), s = sin(theta);
		return Pose2(c * x_ - s * y_, s * x_ + c * y_, theta + theta_);
	}

/* ************************************************************************* */

} // namespace gtsam
