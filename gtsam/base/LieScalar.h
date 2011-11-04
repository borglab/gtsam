/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieScalar.h
 * @brief A wrapper around scalar providing Lie compatibility
 * @author Kai Ni
 */

#pragma once

#include <gtsam/base/Lie.h>

namespace gtsam {

	/**
	 * LieScalar is a wrapper around double to allow it to be a Lie type
	 */
	struct LieScalar {

		/** default constructor - should be unnecessary */
		LieScalar() {}

		/** wrap a double */
		LieScalar(double d) : d_(d) {}

		/** access the underlying value */
		double value() const { return d_; }

		/** print @param s optional string naming the object */
		inline void print(const std::string& name="") const {
	    std::cout << name << ": " << d_ << std::endl;
		}

		/** equality up to tolerance */
		inline bool equals(const LieScalar& expected, double tol=1e-5) const {
			return fabs(expected.d_ - d_) <= tol;
		}

		/**
		 * Returns dimensionality of the tangent space
		 * with member and static versions
		 */
		inline size_t dim() const { return 1; }
		inline static size_t Dim() { return 1; }

		/**
		 * Returns Exponential map update of T
		 * Default implementation calls global binary function
		 */
		inline LieScalar expmap(const Vector& v) const { return LieScalar(d_ + v(0)); }

		/** expmap around identity */
		static inline LieScalar Expmap(const Vector& v) { return LieScalar(v(0)); }

		/**
		 * Returns Log map
		 * Default Implementation calls global binary function
		 */
		inline Vector logmap(const LieScalar& lp) const {
			return Vector_(1, lp.d_ - d_);
		}

		/** Logmap around identity - just returns with default cast back */
		static inline Vector Logmap(const LieScalar& p) { return Vector_(1, p.d_); }

		inline LieScalar between(const LieScalar& t2) const { return LieScalar(t2.value() - d_); }

		/** compose with another object */
		inline LieScalar compose(const LieScalar& t2) const { return LieScalar(t2.value() + d_); }

		/** invert the object and yield a new one */
		inline LieScalar inverse() const { return LieScalar(-d_); }

		// Manifold requirements

		inline LieScalar retract(const Vector& v) const { return expmap(v); }

		/** expmap around identity */
		inline static LieScalar Retract(const Vector& v) { return Expmap(v); }

		/**
		 * Returns inverse retraction
		 */
		inline Vector unretract(const LieScalar& t2) const { return logmap(t2); }

		/** Unretract around identity */
		inline static Vector Unretract(const LieScalar& t) { return Logmap(t); }

	private:
	    double d_;
	};
} // \namespace gtsam
