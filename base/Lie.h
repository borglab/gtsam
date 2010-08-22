/**
 * @file Lie.h
 * @brief Base class and basic functions for Lie types
 * @author Richard Roberts
 * @author Alex Cunningham
 */

#pragma once

#include <string>

#include <gtsam/base/Matrix.h>

namespace gtsam {

  /**
   * These core global functions can be specialized by new Lie types
   * for better performance.
   */

  /* Exponential map about identity */
  template<class T>
  T expmap(const Vector& v) { return T::Expmap(v); }

  /* Logmap (inverse exponential map) about identity */
  template<class T>
  Vector logmap(const T& p) { return T::Logmap(p); }

  /** Compute l1 s.t. l2=l1*l0 */
  template<class T>
  inline T between(const T& l1, const T& l2) { return compose(inverse(l1),l2); }

  /** Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp */
  template<class T>
  inline Vector logmap(const T& l0, const T& lp) { return logmap(between(l0,lp)); }

  /** Exponential map centered at l0, s.t. exp(t,d) = t*exp(d) */
  template<class T>
  inline T expmap(const T& t, const Vector& d) { return compose(t,expmap<T>(d)); }

  /**
   * Base class for Lie group type
   * This class uses the Curiously Recurring Template design pattern to allow
   * for static polymorphism.
   *
   * T is the derived Lie type, like Point2, Pose3, etc.
   *
   * By convention, we use capital letters to designate a static function
   *
   * FIXME: Need to find a way to check for actual implementations in T
   * so that there are no recursive function calls.  This could be handled
   * by not using the same name
   */
  template <class T>
  class Lie {
  public:

    /**
     * Returns dimensionality of the tangent space
     */
    inline size_t dim() const {
    	return static_cast<const T*>(this)->dim();
    }

    /**
     * Returns Exponential map update of T
     * Default implementation calls global binary function
     */
    T expmap(const Vector& v) const;

    /** expmap around identity */
    static T Expmap(const Vector& v) {
    	return T::Expmap(v);
    }

    /**
     * Returns Log map
     * Default Implementation calls global binary function
     */
    Vector logmap(const T& lp) const;

    /** Logmap around identity */
    static Vector Logmap(const T& p) {
    	return T::Logmap(p);
    }

    /** compose with another object */
    inline T compose(const T& p) const {
    	return static_cast<const T*>(this)->compose(p);
    }

    /** invert the object and yield a new one */
    inline T inverse() const {
    	return static_cast<const T*>(this)->inverse();
    }

  };
  
  /** get the dimension of an object with a global function */
  template<class T>
  inline size_t dim(const T& object) {
	  return object.dim();
  }

  /** compose two Lie types */
  template<class T>
  inline T compose(const T& p1, const T& p2) {
	  return p1.compose(p2);
  }

  /** invert an object */
  template<class T>
  inline T inverse(const T& p) {
	  return p.inverse();
  }

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

} // namespace gtsam
