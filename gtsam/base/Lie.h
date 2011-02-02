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
 *
 * The necessary functions to implement for Lie are defined
 * below with additional details as to the interface.  The
 * concept checking function in class Lie will check whether or not
 * the function exists and throw compile-time errors.
 *
 * Returns dimensionality of the tangent space
 * 		inline size_t dim() const;
 *
 * Returns Exponential map update of T
 * A default implementation of expmap(*this, lp) is available:
 * expmap_default()
 * 		T expmap(const Vector& v) const;
 *
 * expmap around identity
 * 		static T Expmap(const Vector& v);
 *
 * Returns Log map
 * A default implementation of logmap(*this, lp) is available:
 * logmap_default()
 * 		Vector logmap(const T& lp) const;
 *
 * Logmap around identity
 * 		static Vector Logmap(const T& p);
 *
 * Compute l0 s.t. l2=l1*l0, where (*this) is l1
 * A default implementation of between(*this, lp) is available:
 * between_default()
 * 		T between(const T& l2) const;
 *
 * compose with another object
 * 		T compose(const T& p) const;
 *
 * invert the object and yield a new one
 * 		T inverse() const;
 *
 */


#pragma once

#include <string>

#include <gtsam/base/Matrix.h>

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
	 * Base class for Lie group type
	 * This class uses the Curiously Recurring Template design pattern to allow for
	 * concept checking using a private function.
	 *
	 * T is the derived Lie type, like Point2, Pose3, etc.
	 *
	 * By convention, we use capital letters to designate a static function
	 */
	template <class T>
	class Lie {
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
			 * Returns Exponential map update of T
			 * Default implementation calls global binary function
			 */
			T expmap_ret = t.expmap(gtsam::zero(dim_ret));

			/** expmap around identity */
			T expmap_identity_ret = T::Expmap(gtsam::zero(dim_ret));

			/**
			 * Returns Log map
			 * Default Implementation calls global binary function
			 */
			Vector logmap_ret = t.logmap(t2);

			/** Logmap around identity */
			Vector logmap_identity_ret = T::Logmap(t);

			/** Compute l0 s.t. l2=l1*l0, where (*this) is l1 */
			T between_ret = t.between(t2);

			/** compose with another object */
			T compose_ret = t.compose(t2);

			/** invert the object and yield a new one */
			T inverse_ret = t.inverse();
		}

	};

	/** Call print on the object */
	template<class T>
	inline void print(const T& object, const std::string& s = "") {
		object.print(s);
	}

	/** Call equal on the object */
	template<class T>
	inline bool equal(const T& obj1, const T& obj2, double tol) {
		return obj1.equals(obj2, tol);
	}

	/** Call equal on the object without tolerance (use default tolerance) */
	template<class T>
	inline bool equal(const T& obj1, const T& obj2) {
		return obj1.equals(obj2);
	}

	/**
	 *  Three term approximation of the Baker�Campbell�Hausdorff formula
	 *  In non-commutative Lie groups, when composing exp(Z) = exp(X)exp(Y)
	 *  it is not true that Z = X+Y. Instead, Z can be calculated using the BCH
	 *  formula: Z = X + Y + [X,Y]/2 + [X-Y,[X,Y]]/12 - [Y,[X,[X,Y]]]/24
	 *  http://en.wikipedia.org/wiki/Baker�Campbell�Hausdorff_formula
	 */
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
		return expm(xhat,K);
	}

	/**
	 * function wrappers for full versions of expmap/logmap
	 * these will default simple types to using the existing expmap/logmap,
	 * but more complex ones can be specialized to use improved versions
	 *
	 * TODO: replace this approach with a naming scheme that doesn't call
	 * non-expmap operations "expmap" - use same approach, but with "update"
	 */

  /** unary versions */
	template<class T>
	T ExpmapFull(const Vector& xi) { return T::Expmap(xi); }
	template<class T>
  Vector LogmapFull(const T& p) { return T::Logmap(p); }

  /** binary versions */
	template<class T>
  T expmapFull(const T& t, const Vector& v) { return t.expmap(v); }
	template<class T>
  Vector logmapFull(const T& t, const T& p2) { return t.logmap(p2); }

} // namespace gtsam
