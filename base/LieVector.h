/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieVector.h
 * @brief A wrapper around vector providing Lie compatibility
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

	/**
	 * LieVector is a wrapper around vector to allow it to be a Lie type
	 */
	struct LieVector : public Vector, public Lie<LieVector>, Testable<LieVector> {

		/** default constructor - should be unnecessary */
		LieVector() {}

		/** initialize from a normal vector */
		LieVector(const Vector& v) : Vector(v) {}

		/** wrap a double */
		LieVector(double d) : Vector(Vector_(1, d)) {}

		/** constructor with size and initial data, row order ! */
		LieVector(size_t m, const double* const data);

		/** Specify arguments directly, as in Vector_() - always force these to be doubles */
		LieVector(size_t m, ...);

		/** get the underlying vector */
		inline Vector vector() const {
			return static_cast<Vector>(*this);
		}

		/** print @param s optional string naming the object */
		inline void print(const std::string& name="") const {
			gtsam::print(vector(), name);
		}

		/** equality up to tolerance */
		inline bool equals(const LieVector& expected, double tol=1e-5) const {
			return gtsam::equal(vector(), expected.vector(), tol);
		}

	    /**
	     * Returns dimensionality of the tangent space
	     */
	    inline size_t dim() const { return this->size(); }

	    /**
	     * Returns Exponential map update of T
	     * Default implementation calls global binary function
	     */
	    inline LieVector expmap(const Vector& v) const { return LieVector(vector() + v); }

	    /** expmap around identity */
	    static inline LieVector Expmap(const Vector& v) { return LieVector(v); }

	    /**
	     * Returns Log map
	     * Default Implementation calls global binary function
	     */
	    inline Vector logmap(const LieVector& lp) const {
//	    	return Logmap(between(lp)); // works
	    	return lp.vector() - vector();
	    }

	    /** Logmap around identity - just returns with default cast back */
	    static inline Vector Logmap(const LieVector& p) { return p; }

	    /** compose with another object */
	    inline LieVector compose(const LieVector& p) const {
	    	return LieVector(vector() + p);
	    }

	    /** between operation */
	    inline LieVector between(const LieVector& l2) const {
	    	return LieVector(l2.vector() - vector());
	    }

	    /** invert the object and yield a new one */
	    inline LieVector inverse() const {
	    	return LieVector(-1.0 * vector());
	    }
	};
} // \namespace gtsam
