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
	Pose2 Pose2::exmap(const Vector& v) const {
		return Pose2(x_ + v(0), y_ + v(1), theta_ + v(2));
	}

	/* ************************************************************************* */
	bool Pose2::equals(const Pose2& q, double tol) const {
		return (fabs(x_ - q.x_) < tol && fabs(y_ - q.y_) < tol && fabs(theta_
				- q.theta_) < tol);
	}

	/* ************************************************************************* */
	bool assert_equal(const Pose2& A, const Pose2& B, double tol) {
		if (A.equals(B, tol)) return true;
		printf("not equal:\n");
		A.print("A");
		B.print("B");
		return false;
	}

/* ************************************************************************* */

} // namespace gtsam
