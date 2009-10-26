/**
 *@file  Pose3.h
 *@brief 3D Pose
 */

// \callgraph

#pragma once

#include "Point3.h"
#include "Rot3.h"
#include "Testable.h"

namespace gtsam {

	/** A 3D pose (R,t) : (Rot3,Point3) */
	class Pose3 : Testable<Pose3> {
	private:
		Rot3 R_;
		Point3 t_;

	public:

		/**
		 * Default constructor is origin
		 */
		Pose3() {}

		/**
		 * Copy constructor
		 */
		Pose3(const Pose3& pose) :
			R_(pose.R_), t_(pose.t_) {
		}

		/**
		 * Construct from R,t
		 */
		Pose3(const Rot3& R, const Point3& t) :
			R_(R), t_(t) {
		}

		/** Constructor from 4*4 matrix */
		Pose3(const Matrix &T) :
			R_(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0),
					T(2, 1), T(2, 2)), t_(T(0, 3), T(1, 3), T(2, 3)) {
		}

		/** Constructor from 12D vector */
		Pose3(const Vector &V) :
			R_(V(0), V(3), V(6), V(1), V(4), V(7), V(2), V(5), V(8)),
			t_(V(9), V(10),V(11)) {
		}

		/** print with optional string */
		void print(const std::string& s = "") const;

		/** assert equality up to a tolerance */
		bool equals(const Pose3& pose, double tol = 1e-9) const;

		const Rot3& rotation() const {
			return R_;
		}

		const Point3& translation() const {
			return t_;
		}

		/** return DOF, dimensionality of tangent space = 6 */
		size_t dim() const {
			return 6;
		}

		/** Given 6-dim tangent vector, create new pose */
		Pose3 exmap(const Vector& d) const;

		/** inverse transformation */
		Pose3 inverse() const;

		/** composition */
		inline Pose3 operator*(const Pose3& B) const {
			return Pose3(R_*B.R_, t_ + R_*B.t_);
		}

		/** return 12D vectorized form (column-wise) */
		Vector vector() const;

		/** convert to 4*4 matrix */
		Matrix matrix() const;

		/** transforms */
		Pose3 transformPose_to(const Pose3& transform) const;

		/** friends */
		friend Point3 transform_from(const Pose3& pose, const Point3& p);
		friend Point3 transform_to(const Pose3& pose, const Point3& p);
		friend Pose3 composeTransform(const Pose3& current, const Pose3& target);

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(R_);
			ar & BOOST_SERIALIZATION_NVP(t_);
		}
	}; // Pose3 class

	/**
	 * Finds a transform from the current frame dTx to the target frame sTx
	 */
	inline Pose3 composeTransform(const Pose3& dTx, const Pose3& sTx) {
		return dTx * sTx.inverse();
	}

	/** receives the point in Pose coordinates and transforms it to world coordinates */
	Point3 transform_from(const Pose3& pose, const Point3& p);
	Matrix Dtransform_from1(const Pose3& pose, const Point3& p);
	Matrix Dtransform_from2(const Pose3& pose); // does not depend on p !

	/** receives the point in world coordinates and transforms it to Pose coordinates */
	Point3 transform_to(const Pose3& pose, const Point3& p);
	Matrix Dtransform_to1(const Pose3& pose, const Point3& p);
	Matrix Dtransform_to2(const Pose3& pose); // does not depend on p !

	/** direct measurement of a pose */
	Vector hPose(const Vector& x);

	/**
	 * derivative of direct measurement
	 * 12*6, entry i,j is how measurement error will change
	 */
	Matrix DhPose(const Vector& x);

} // namespace gtsam
