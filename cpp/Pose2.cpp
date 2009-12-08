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
	Point2 transform_to(const Pose2& pose, const Point2& point) {
		double dx = point.x()-pose.x(), dy = point.y()-pose.y();
		double ct=cos(pose.theta()), st=sin(pose.theta());
		return Point2(ct*dx + st*dy, -st*dx + ct*dy);
	}

	// TODO, have a combined function that returns both function value and derivative
	Matrix Dtransform_to1(const Pose2& pose, const Point2& point) {
		double dx = point.x()-pose.x(), dy = point.y()-pose.y();
		double ct=cos(pose.theta()), st=sin(pose.theta());
		double transformed_x = ct*dx + st*dy, transformed_y = -st*dx + ct*dy;
		return Matrix_(2,3,
				-ct, -st,  transformed_y,
				 st, -ct, -transformed_x
				);
	}

	Matrix Dtransform_to2(const Pose2& pose, const Point2& point) {
		double ct=cos(pose.theta()), st=sin(pose.theta());
		return Matrix_(2,2,
				 ct,  st,
				-st,  ct
				);
	}

	/* ************************************************************************* */
	Pose2 between(const Pose2& p1, const Pose2& p2) {
		double dx = p2.x()-p1.x(), dy = p2.y()-p1.y();
		double ct=cos(p1.theta()), st=sin(p1.theta());
		return Pose2(ct*dx + st*dy, -st*dx + ct*dy, p2.theta()-p1.theta());
	}

	Matrix Dbetween1(const Pose2& p1, const Pose2& p2) {
		double dx = p2.x()-p1.x(), dy = p2.y()-p1.y();
		double ct=cos(p1.theta()), st=sin(p1.theta());
		double transformed_x = ct*dx + st*dy, transformed_y = -st*dx + ct*dy;
		return Matrix_(3,3,
				-ct, -st,  transformed_y,
				 st, -ct, -transformed_x,
				0.0, 0.0, -1.0
				);
	}

	Matrix Dbetween2(const Pose2& p1, const Pose2& p2) {
		double ct=cos(p1.theta()), st=sin(p1.theta());
		return Matrix_(3,3,
				 ct,  st, 0.0,
				-st,  ct, 0.0,
				0.0, 0.0, 1.0
				);
	}

	/* ************************************************************************* */

} // namespace gtsam
