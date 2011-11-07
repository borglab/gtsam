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
 * @author Richard Roberts
 * @author Alex Cunningham
 *
 * The necessary functions to implement for Manifold are defined
 * below with additional details as to the interface.  The
 * concept checking function in class Manifold will check whether or not
 * the function exists and throw compile-time errors.
 *
 * Returns dimensionality of the tangent space
 * 		inline size_t dim() const;
 *
 * Returns Retraction update of T
 * 		T retract(const Vector& v) const;
 *
 * Returns inverse retraction operation
 * 		Vector localCoordinates(const T& lp) const;
 *
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
	 * T is the Manifold type, like Point2, Pose3, etc.
	 *
	 * By convention, we use capital letters to designate a static function
	 */
	template <class T>
	class ManifoldConcept {
	private:
		/** concept checking function - implement the functions this demands */
		static void concept_check(const T& t) {

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
