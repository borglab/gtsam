/*
 * Lie.h
 *
 *  Created on: Jan 5, 2010
 *      Author: Richard Roberts
 */

#ifndef LIE_H_
#define LIE_H_

#include <string>
#include "Vector.h"

namespace gtsam {

  template<class T>
  T expmap(const Vector& v);  /* Exponential map about identity */

  // Syntactic sugar
  template<class T>
  inline T operator*(const T& l1, const T& l0) { return compose(l1, l0); }


  // The following functions may be overridden in your own class file
  // with more efficient versions if possible.

  // Compute l1 s.t. l2=l1*l0
  template<class T>
  inline T between(const T& l0, const T& l2) { return l2*inverse(l0); }

  // Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp
  template<class T>
  inline Vector logmap(const T& l0, const T& lp) { return logmap(between(l0,lp)); }

  /* Exponential map centered at l0, s.t. exp(l0,v) = exp(v)*l0 */
  template<class T>
  inline T expmap(const T& l0, const Vector& v) { return expmap<T>(v)*l0; }

  /**
   * Base class for Lie group type
   */
  template <class T>
  class Lie {
  public:

    /**
     * Returns dimensionality of the tangent space
     */
    size_t dim() const;

    /**
     * Returns Exponential mapy
     */
    T expmap(const Vector& v) const;

    /**
     * Returns Log map
     */
    Vector logmap(const T& lp) const;

  };
  
  /** Call print on the object */
  template<class T>
  inline void print_(const T& object, const std::string& s = "") {
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
    return obj1.equal(obj2);
  }

  // Vector Group operations
  inline Vector compose(const Vector& p1,const Vector& p2) { return p1+p2;}
  inline Vector inverse(const Vector& p) { return -p;}
  inline Vector between(const Vector& p1,const Vector& p2) { return p2-p1;}

  // Vector is a trivial Lie Group
  template<> inline Vector expmap(const Vector& d) { return d;}
  template<> inline Vector expmap(const Vector& p,const Vector& d) { return p+d;}
  inline Vector logmap(const Vector& p) { return p;}
  inline Vector logmap(const Vector& p1,const Vector& p2) { return p2-p1;}

}


#endif /* LIE_H_ */
