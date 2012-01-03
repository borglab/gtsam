/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Manifold.h
 * @brief Base class and basic functions for Manifold types
 * @author Alex Cunningham
 */

#pragma once

#include <string>
#include <gtsam/base/Matrix.h>

namespace gtsam {

/**
 * Concept check class for Manifold types
 * Requires a mapping between a linear tangent space and the underlying
 * manifold, of which Lie is a specialization.
 *
 * The necessary functions to implement for Manifold are defined
 * below with additional details as to the interface.  The
 * concept checking function in class Manifold will check whether or not
 * the function exists and throw compile-time errors.
 *
 * A manifold defines a space in which there is a notion of a linear tangent space
 * that can be centered around a given point on the manifold.  These nonlinear
 * spaces may have such properties as wrapping around (as is the case with rotations),
 * which might make linear operations on parameters not return a viable element of
 * the manifold.
 *
 * We perform optimization by computing a linear delta in the tangent space of the
 * current estimate, and then apply this change using a retraction operation, which
 * maps the change in tangent space back to the manifold itself.
 *
 * There may be multiple possible retractions for a given manifold, which can be chosen
 * between depending on the computational complexity.  The important criteria for
 * the creation for the retract and localCoordinates functions is that they be
 * inverse operations.
 *
 * Returns dimensionality of the tangent space, which may be smaller than the number
 * of nonlinear parameters.
 * 		size_t dim() const;
 *
 * Returns a new T that is a result of updating *this with the delta v after pulling
 * the updated value back to the manifold T.
 * 		T retract(const Vector& v) const;
 *
 * Returns the linear coordinates of lp in the tangent space centered around *this.
 * 		Vector localCoordinates(const T& lp) const;
 *
 * By convention, we use capital letters to designate a static function
 * @tparam T is a Lie type, like Point2, Pose3, etc.
 */
template <class T>
class ManifoldConcept {
private:
	/** concept checking function - implement the functions this demands */
	static T concept_check(const T& t) {

		/** assignment */
		T t2 = t;

		/**
		 * Returns dimensionality of the tangent space
		 */
		size_t dim_ret = t.dim();

		/**
		 * Returns Retraction update of T
		 */
		T retract_ret = t.retract(gtsam::zero(dim_ret));

		/**
		 * Returns local coordinates of another object
		 */
		Vector localCoords_ret = t.localCoordinates(t2);

		return retract_ret;
	}
};

} // namespace gtsam

/**
 * Macros for using the ManifoldConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_MANIFOLD_INST(T) template class gtsam::ManifoldConcept<T>;
#define GTSAM_CONCEPT_MANIFOLD_TYPE(T) typedef gtsam::ManifoldConcept<T> _gtsam_ManifoldConcept_##T;
