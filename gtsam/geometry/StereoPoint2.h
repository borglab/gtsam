/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StereoPoint2.h
 * @brief A 2D stereo point (uL,uR,v)
 * @date Jan 26, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

	/**
	 * A 2D stereo point, v will be same for rectified images
	 * @addtogroup geometry
	 * \nosubgrouping
	 */
	class StereoPoint2 : public DerivedValue<StereoPoint2> {
	public:
		static const size_t dimension = 3;
	private:
		double uL_, uR_, v_;

	public:

		/// @name Standard Constructors
		/// @{

		/** default constructor */
		StereoPoint2() :
			uL_(0), uR_(0), v_(0) {
		}

		/** constructor */
		StereoPoint2(double uL, double uR, double v) :
			uL_(uL), uR_(uR), v_(v) {
		}

    /// @}
    /// @name Testable
    /// @{

		/** print */
		void print(const std::string& s="") const;

		/** equals */
		bool equals(const StereoPoint2& q, double tol=1e-9) const {
			return (fabs(uL_ - q.uL_) < tol && fabs(uR_ - q.uR_) < tol && fabs(v_
					- q.v_) < tol);
		}

    /// @}
    /// @name Group
    /// @{

		/// identity
		inline static StereoPoint2 identity() { return StereoPoint2(); }

		/// inverse
		inline StereoPoint2 inverse() const {
			return StereoPoint2()- (*this);
		}

		/// "Compose", just adds the coordinates of two points.
		inline StereoPoint2 compose(const StereoPoint2& p1) const {
			return *this + p1;
		}

		/// add two stereo points
		StereoPoint2 operator+(const StereoPoint2& b) const {
			return StereoPoint2(uL_ + b.uL_, uR_ + b.uR_, v_ + b.v_);
		}

		/// subtract two stereo points
		StereoPoint2 operator-(const StereoPoint2& b) const {
			return StereoPoint2(uL_ - b.uL_, uR_ - b.uR_, v_ - b.v_);
		}

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes */
    inline static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    inline size_t dim() const { return dimension; }

  	/// Updates a with tangent space delta
		inline StereoPoint2 retract(const Vector& v) const { return compose(Expmap(v)); }

		/// Returns inverse retraction
		inline Vector localCoordinates(const StereoPoint2& t2) const { return Logmap(between(t2)); }

    /// @}
    /// @name Lie Group
    /// @{

		/** Exponential map around identity - just create a Point2 from a vector */
		static inline StereoPoint2 Expmap(const Vector& d) {
			return StereoPoint2(d(0), d(1), d(2));
		}

		/** Log map around identity - just return the Point2 as a vector */
		static inline Vector Logmap(const StereoPoint2& p) {
			return p.vector();
		}

    /** The difference between another point and this point */
    inline StereoPoint2 between(const StereoPoint2& p2) const {
      return gtsam::between_default(*this, p2);
    }

		/// @}
		/// @name Standard Interface
		/// @{

		/** convert to vector */
		Vector vector() const {
			return Vector_(3, uL_, uR_, v_);
		}

		/** convenient function to get a Point2 from the left image */
		inline Point2 point2(){
			return Point2(uL_, v_);
		}

	private:

		/// @}
		/// @name Advanced Interface
		/// @{

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("StereoPoint2",
					boost::serialization::base_object<Value>(*this));
			ar & BOOST_SERIALIZATION_NVP(uL_);
			ar & BOOST_SERIALIZATION_NVP(uR_);
			ar & BOOST_SERIALIZATION_NVP(v_);
		}

		/// @}

	};

}
