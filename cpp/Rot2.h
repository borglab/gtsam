/*
 * Rot2.h
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#ifndef ROT2_H_
#define ROT2_H_

#include "Testable.h"
#include "Point2.h"
#include "Matrix.h"

namespace gtsam {

	/* Rotation matrix */
	class Rot2: Testable<Rot2> {
	private:
		/** we store cos(theta) and sin(theta) */
		double c_, s_;

		/** private constructor from cos/sin */
		Rot2(double c, double s) :
			c_(c), s_(s) {
		}

	public:

		/** default constructor, zero rotation */
		Rot2() : c_(1.0), s_(0.0) {}

		/** constructor from angle */
		Rot2(double theta) : c_(cos(theta)), s_(sin(theta)) {}

		/** return angle */
		inline double angle() const {	return atan2(s_, c_);	}

		/** print */
		void print(const std::string& s = "theta") const;

		/** equals with an tolerance */
		bool equals(const Rot2& R, double tol = 1e-9) const;

		/** return DOF, dimensionality of tangent space */
		inline size_t dim() const { return 1;}

		/** Given 1-dim tangent vector, create new rotation */
		Rot2 exmap(const Vector& d) const;

		/** return vectorized form (column-wise)*/
		inline Vector vector() const { return Vector_(2,c_,s_);}

		/** return 2*2 rotation matrix */
		Matrix matrix() const {
			double r[] = { c_, -s_, s_, c_ };
			return Matrix_(2, 2, r);
		}

		/** return 3*3 transpose (inverse) rotation matrix   */
		Matrix transpose() const {
			double r[] = { c_, s_, -s_, c_ };
			return Matrix_(2, 2, r);
		}

		/** inverse transformation  */
		Rot2 inverse() const { return Rot2(c_, -s_);}

		/** composition via sum and difference formulas */
		inline Rot2 operator*(const Rot2& R) const {
			return Rot2(c_ * R.c_ - s_ * R.s_, s_ * R.c_ + c_ * R.s_);
		}

		/**  rotate from rotated to world, syntactic sugar = R*p  */
		inline Point2 operator*(const Point2& p) const {
			return Point2(c_ * p.x() - s_ * p.y(), s_ * p.x() + c_ * p.y());
		}

		/** rotate from world to rotated = R'*p */
		Point2 unrotate(const Point2& p) const {
			return Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
		}

		/** friends */
		friend Matrix Dunrotate1(const Rot2& R, const Point2& p);

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(c_);
			ar & BOOST_SERIALIZATION_NVP(s_);
		}
	};

	/**
	 * Update Rotation with incremental rotation
	 * @param v a vector of incremental angle
	 * @param R a 2D rotation
	 * @return incremental rotation matrix
	 */
	Rot2 exmap(const Rot2& R, const Vector& v);

	/**
	 * rotate point from rotated coordinate frame to
	 * world = R*p
	 */
	Point2 rotate(const Rot2& R, const Point2& p);
	Matrix Drotate1(const Rot2& R, const Point2& p);
	Matrix Drotate2(const Rot2& R); // does not depend on p !

	/**
	 * rotate point from world to rotated
	 * frame = R'*p
	 */
	Point2 unrotate(const Rot2& R, const Point2& p);
	Matrix Dunrotate1(const Rot2& R, const Point2& p);
	Matrix Dunrotate2(const Rot2& R); // does not depend on p !

} // gtsam

#endif /* ROT2_H_ */
