/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2.h
 * @brief 2D rotation
 * @date Dec 9, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <boost/optional.hpp>

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

	/**
	 * Rotation matrix
	 * NOTE: the angle theta is in radians unless explicitly stated
	 * @addtogroup geometry
	 * \nosubgrouping
	 */
	class Rot2 : public DerivedValue<Rot2> {

	public:
		/** get the dimension by the type */
		static const size_t dimension = 1;

	private:

		/** we store cos(theta) and sin(theta) */
		double c_, s_;


		/** normalize to make sure cos and sin form unit vector */
		Rot2& normalize();

		/** private constructor from cos/sin */
		inline Rot2(double c, double s) :
				c_(c), s_(s) {
		}

	public:

    /// @name Constructors and named constructors
    /// @{

		/** default constructor, zero rotation */
		Rot2() :
				c_(1.0), s_(0.0) {
		}

		/// Constructor from angle in radians == exponential map at identity
		Rot2(double theta) :
				c_(cos(theta)), s_(sin(theta)) {
		}

		/// Named constructor from angle in radians
		static Rot2 fromAngle(double theta) {
			return Rot2(theta);
		}

		/// Named constructor from angle in degrees
		static Rot2 fromDegrees(double theta) {
			const double degree = M_PI / 180;
			return fromAngle(theta * degree);
		}

		/// Named constructor from cos(theta),sin(theta) pair, will *not* normalize!
		static Rot2 fromCosSin(double c, double s);

		/**
		 * Named constructor with derivative
		 * Calculate relative bearing to a landmark in local coordinate frame
		 * @param d 2D location of landmark
		 * @param H optional reference for Jacobian
		 * @return 2D rotation \f$ \in SO(2) \f$
		 */
		static Rot2 relativeBearing(const Point2& d, boost::optional<Matrix&> H =
				boost::none);

		/** Named constructor that behaves as atan2, i.e., y,x order (!) and normalizes */
		static Rot2 atan2(double y, double x);

	  /// @}
    /// @name Testable
    /// @{

		/** print */
		void print(const std::string& s = "theta") const;

		/** equals with an tolerance */
		bool equals(const Rot2& R, double tol = 1e-9) const;

	  /// @}
	  /// @name Group
	  /// @{

		/** identity */
		inline static Rot2 identity() {	return Rot2(); }

		/** The inverse rotation - negative angle */
		Rot2 inverse() const {
			return Rot2(c_, -s_);
		}

		/** Compose - make a new rotation by adding angles */
		inline Rot2 compose(const Rot2& R1, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			if (H1) *H1 = eye(1);
			if (H2) *H2 = eye(1);
			return *this * R1;
		}

		/** Compose - make a new rotation by adding angles */
		Rot2 operator*(const Rot2& R) const {
			return fromCosSin(c_ * R.c_ - s_ * R.s_, s_ * R.c_ + c_ * R.s_);
		}

		/** Between using the default implementation */
		inline Rot2 between(const Rot2& p2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			if (H1) *H1 = -eye(1);
			if (H2) *H2 = eye(1);
			return between_default(*this, p2);
		}

	  /// @}
	  /// @name Manifold
	  /// @{

		/// dimension of the variable - used to autodetect sizes
		inline static size_t Dim() {
			return dimension;
		}

		/// Dimensionality of the tangent space, DOF = 1
		inline size_t dim() const {
			return dimension;
		}

  	/// Updates a with tangent space delta
  	inline Rot2 retract(const Vector& v) const { return *this * Expmap(v); }

  	/// Returns inverse retraction
  	inline Vector localCoordinates(const Rot2& t2) const { return Logmap(between(t2)); }

	  /// @}
	  /// @name Lie Group
	  /// @{

  	///Exponential map at identity - create a rotation from canonical coordinates
		static Rot2 Expmap(const Vector& v) {
			if (zero(v))
				return (Rot2());
			else
				return Rot2::fromAngle(v(0));
		}

		///Log map at identity - return the canonical coordinates of this rotation
		static inline Vector Logmap(const Rot2& r) {
			return Vector_(1, r.theta());
		}

  	/// @}
  	/// @name Group Action on Point2
  	/// @{

    /**
		 * rotate point from rotated coordinate frame to world \f$ p^w = R_c^w p^c \f$
		 */
		Point2 rotate(const Point2& p, boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

		/** syntactic sugar for rotate */
		inline Point2 operator*(const Point2& p) const {
			return rotate(p);
		}

		/**
		 * rotate point from world to rotated frame \f$ p^c = (R_c^w)^T p^w \f$
		 */
		Point2 unrotate(const Point2& p, boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

		/// @}
		/// @name Standard Interface
		/// @{

		/// Creates a unit vector as a Point2
		inline Point2 unit() const {
			return Point2(c_, s_);
		}

		/** return angle (RADIANS) */
		double theta() const {
			return ::atan2(s_, c_);
		}

		/** return angle (DEGREES) */
		double degrees() const {
			const double degree = M_PI / 180;
			return theta() / degree;
		}

		/** return cos */
		inline double c() const {
			return c_;
		}

		/** return sin */
		inline double s() const {
			return s_;
		}

		/** return 2*2 rotation matrix */
		Matrix matrix() const;

		/** return 2*2 transpose (inverse) rotation matrix   */
		Matrix transpose() const;

		/// @}
		/// @name Advanced Interface
		/// @{

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("Rot2",
					boost::serialization::base_object<Value>(*this));
			ar & BOOST_SERIALIZATION_NVP(c_);
			ar & BOOST_SERIALIZATION_NVP(s_);
		}

	  /// @}

	};

} // gtsam
