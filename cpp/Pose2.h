/**
 * @file  Pose2.h
 * @brief 3D Pose
 * @author: Frank Dellaert
 */

// \callgraph

#pragma once

#include "Point2.h"
#include "Matrix.h"
#include "Testable.h"

namespace gtsam {

	/**
	 * A 2D pose (x,y,theta)
	 */
	class Pose2: Testable<Pose2>  {

	private:
		double x_, y_, theta_;

	public:

		/** default constructor = origin */
		Pose2() :
			x_(0), y_(0), theta_(0) {
		} // default is origin

		/** copy constructor */
		Pose2(const Pose2& pose) :
			x_(pose.x_), y_(pose.y_), theta_(pose.theta_) {
		}

		/**
		 * construct from (x,y,theta)
		 * @param x x oordinate
		 * @param y y coordinate
		 * @param theta angle with positive X-axis
		 */
		Pose2(double x, double y, double theta) :
			x_(x), y_(y), theta_(theta) {
		}

		/** construct from rotation and translation */
		Pose2(const Point2& t, double theta) :
			x_(t.x()), y_(t.y()), theta_(theta) {
		}

		/** print with optional string */
		void print(const std::string& s = "") const;

		/** assert equality up to a tolerance */
		bool equals(const Pose2& pose, double tol = 1e-9) const;

    /** get functions for x, y, theta */
		inline double x()     const { return x_;}
		inline double y()     const { return y_;}
		inline double theta() const { return theta_;}

		/** return DOF, dimensionality of tangent space = 3 */
		size_t dim() const {
			return 3;
		}

		/* exponential map */
		Pose2 exmap(const Vector& v) const;

    /** return vectorized form (column-wise)*/
    Vector vector() const;

    /** rotate pose by theta */
    Pose2 rotate(double theta) const;

    // operators
    Pose2 operator+(const Pose2& p2) const {
  		return Pose2(x_ + p2.x_, y_ + p2.y_, theta_ + p2.theta_);
    }

    Pose2 operator-(const Pose2& p2) const {
  		return Pose2(x_ - p2.x_, y_ - p2.y_, theta_ - p2.theta_);
    }
	}; // Pose2

	/**
	 * Return point coordinates in pose coordinate frame
	 */
	Point2 transform_to(const Pose2& pose, const Point2& point);
	Matrix Dtransform_to1(const Pose2& pose, const Point2& point);
	Matrix Dtransform_to2(const Pose2& pose, const Point2& point);

	/**
	 * Return relative pose between p1 and p2, in p1 coordinate frame
	 */
	Pose2 between(const Pose2& p1, const Pose2& p2);
	Matrix Dbetween1(const Pose2& p1, const Pose2& p2);
	Matrix Dbetween2(const Pose2& p1, const Pose2& p2);

} // namespace gtsam
