/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2.h
 * @brief  The most common 5DOF 3D->2D calibration
 * @author Frank Dellaert
 */

/**
 * @addtogroup geometry
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

	/**
	 * @brief The most common 5DOF 3D->2D calibration
	 * @addtogroup geometry
	 * \nosubgrouping
	 */
	class Cal3_S2 : public DerivedValue<Cal3_S2> {
	private:
		double fx_, fy_, s_, u0_, v0_;

	public:

		typedef boost::shared_ptr<Cal3_S2> shared_ptr;  ///< shared pointer to calibration object

    /// @name Standard Constructors
    /// @{

		/// Create a default calibration that leaves coordinates unchanged
		Cal3_S2() :
			fx_(1), fy_(1), s_(0), u0_(0), v0_(0) {
		}

		/// constructor from doubles
		Cal3_S2(double fx, double fy, double s, double u0, double v0) :
			fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0) {
		}

    Cal3_S2(const Vector &d): fx_(d(0)), fy_(d(1)), s_(d(2)), u0_(d(3)), v0_(d(4)){}


		/**
		 * Easy constructor, takes fov in degrees, asssumes zero skew, unit aspect
		 * @param fov field of view in degrees
		 * @param w image width
		 * @param h image height
		 */
		Cal3_S2(double fov, int w, int h);

    /// @}
    /// @name Advanced Constructors
    /// @{

		/// load calibration from location (default name is calibration_info.txt)
		Cal3_S2(const std::string &path);

    /// @}
		/// @name Testable
		/// @{

		/// print with optional string
		void print(const std::string& s = "Cal3_S2") const;

		/// Check if equal up to specified tolerance
		bool equals(const Cal3_S2& K, double tol = 10e-9) const;

    /// @}
    /// @name Standard Interface
    /// @{

		/// focal length x
		inline double fx() const { return fx_;}

		/// focal length x
		inline double fy() const { return fy_;}

		/// skew
		inline double skew() const { return s_;}

		/// image center in x
		inline double px() const { return u0_;}

		/// image center in y
		inline double py() const { return v0_;}

		/// return the principal point
		Point2 principalPoint() const {
			return Point2(u0_, v0_);
		}

		/// vectorized form (column-wise)
		Vector vector() const {
			double r[] = { fx_, fy_, s_, u0_, v0_ };
			Vector v(5);
			std::copy(r, r + 5, v.data());
			return v;
		}

		/// return calibration matrix K
		Matrix matrix() const {
			return Matrix_(3, 3, fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0);
		}

    /// return inverted calibration matrix inv(K)
    Matrix matrix_inverse() const {
      const double fxy = fx_*fy_, sv0 = s_*v0_, fyu0 = fy_*u0_;
      return Matrix_(3, 3,
          1.0/fx_, -s_/fxy, (sv0-fyu0)/fxy,
          0.0,     1.0/fy_, -v0_/fy_,
          0.0, 0.0, 1.0);
    }

		/**
		 * convert intrinsic coordinates xy to image coordinates uv
		 * with optional derivatives
		 */
		Point2 uncalibrate(const Point2& p, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const;

		/// convert image coordinates uv to intrinsic coordinates xy
		Point2 calibrate(const Point2& p) const {
			const double u = p.x(), v = p.y();
			return Point2((1 / fx_) * (u - u0_ - (s_ / fy_) * (v - v0_)),
			              (1 / fy_)	* (v - v0_));
		}

		/// @}
		/// @name Manifold
		/// @{

		/// return DOF, dimensionality of tangent space
		inline size_t dim() const {
			return 5;
		}

		/// return DOF, dimensionality of tangent space
		static size_t Dim() {
			return 5;
		}

		/// Given 5-dim tangent vector, create new calibration
		inline Cal3_S2 retract(const Vector& d) const {
			return Cal3_S2(fx_ + d(0), fy_ + d(1), s_ + d(2), u0_ + d(3), v0_ + d(4));
		}

		/// Unretraction for the calibration
		Vector localCoordinates(const Cal3_S2& T2) const {
			return vector() - T2.vector();
		}

    /// @}
    /// @name Advanced Interface
    /// @{

	private:

		/// Serialization function
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("Cal3_S2",
					boost::serialization::base_object<Value>(*this));
			ar & BOOST_SERIALIZATION_NVP(fx_);
			ar & BOOST_SERIALIZATION_NVP(fy_);
			ar & BOOST_SERIALIZATION_NVP(s_);
			ar & BOOST_SERIALIZATION_NVP(u0_);
			ar & BOOST_SERIALIZATION_NVP(v0_);
		}

		/// @}

	};



	typedef boost::shared_ptr<Cal3_S2> shared_ptrK;	///< shared pointer to calibration object

} // gtsam
