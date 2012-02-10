/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Lie.h
 * @brief Base class and basic functions for Lie types
 * @author Richard Roberts
 * @author Alex Cunningham
 */


#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Group.h>

namespace gtsam {

/**
 * These core global functions can be specialized by new Lie types
 * for better performance.
 */

/** Compute l0 s.t. l2=l1*l0 */
template<class T>
inline T between_default(const T& l1, const T& l2) {return l1.inverse().compose(l2);}

/** Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp */
template<class T>
inline Vector logmap_default(const T& l0, const T& lp) {return T::Logmap(l0.between(lp));}

/** Exponential map centered at l0, s.t. exp(t,d) = t*exp(d) */
template<class T>
inline T expmap_default(const T& t, const Vector& d) {return t.compose(T::Expmap(d));}

/**
 * Concept check class for Lie group type
 *
 * This concept check provides a specialization on the Manifold type,
 * in which the Manifolds represented require an algebra and group structure.
 * All Lie types must also be a Manifold.
 *
 * The necessary functions to implement for Lie are defined
 * below with additional details as to the interface.  The
 * concept checking function in class Lie will check whether or not
 * the function exists and throw compile-time errors.
 *
 * The exponential map is a specific retraction for Lie groups,
 * which maps the tangent space in canonical coordinates back to
 * the underlying manifold.  Because we can enforce a group structure,
 * a retraction of delta v, with tangent space centered at x1 can be performed
 * as x2 = x1.compose(Expmap(v)).
 *
 * Expmap around identity
 * 		static T Expmap(const Vector& v);
 *
 * Logmap around identity
 * 		static Vector Logmap(const T& p);
 *
 * Compute l0 s.t. l2=l1*l0, where (*this) is l1
 * A default implementation of between(*this, lp) is available:
 * between_default()
 * 		T between(const T& l2) const;
 *
 * By convention, we use capital letters to designate a static function
 *
 * @tparam T is a Lie type, like Point2, Pose3, etc.
 */
template <class T>
class LieConcept {
private:
	/** concept checking function - implement the functions this demands */
	static T concept_check(const T& t) {

		/** assignment */
		T t2 = t;

		/**
		 * Returns dimensionality of the tangent space
		 */
		size_t dim_ret = t.dim();

		/** expmap around identity */
		T expmap_identity_ret = T::Expmap(gtsam::zero(dim_ret));

		/** Logmap around identity */
		Vector logmap_identity_ret = T::Logmap(t);

		/** Compute l0 s.t. l2=l1*l0, where (*this) is l1 */
		T between_ret = expmap_identity_ret.between(t2);

		return between_ret;
	}

};

/**
 *  Three term approximation of the Baker�Campbell�Hausdorff formula
 *  In non-commutative Lie groups, when composing exp(Z) = exp(X)exp(Y)
 *  it is not true that Z = X+Y. Instead, Z can be calculated using the BCH
 *  formula: Z = X + Y + [X,Y]/2 + [X-Y,[X,Y]]/12 - [Y,[X,[X,Y]]]/24
 *  http://en.wikipedia.org/wiki/Baker�Campbell�Hausdorff_formula
 */
/// AGC: bracket() only appears in Rot3 tests, should this be used elsewhere?
template<class T>
T BCH(const T& X, const T& Y) {
	static const double _2 = 1. / 2., _12 = 1. / 12., _24 = 1. / 24.;
	T X_Y = bracket(X, Y);
	return X + Y + _2 * X_Y + _12 * bracket(X - Y, X_Y) - _24 * bracket(Y,
			bracket(X, X_Y));
}

/**
 * Declaration of wedge (see Murray94book) used to convert
 * from n exponential coordinates to n*n element of the Lie algebra
 */
template <class T> Matrix wedge(const Vector& x);

/**
 * Exponential map given exponential coordinates
 * class T needs a wedge<> function and a constructor from Matrix
 * @param x exponential coordinates, vector of size n
 * @ return a T
 */
template <class T>
T expm(const Vector& x, int K=7) {
	Matrix xhat = wedge<T>(x);
	return T(expm(xhat,K));
}

} // namespace gtsam

/**
 * Macros for using the LieConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_LIE_INST(T) \
		template class gtsam::ManifoldConcept<T>; \
		template class gtsam::GroupConcept<T>; \
		template class gtsam::LieConcept<T>;

#define GTSAM_CONCEPT_LIE_TYPE(T) \
		typedef gtsam::ManifoldConcept<T> _gtsam_ManifoldConcept_##T; \
		typedef gtsam::GroupConcept<T> _gtsam_GroupConcept_##T; \
		typedef gtsam::LieConcept<T> _gtsam_LieConcept_##T;
