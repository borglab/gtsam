/*
 * Lie.h
 *
 *  Created on: Jan 5, 2010
 *      Author: Richard Roberts
 */

#pragma once

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
  inline T between(const T& l1, const T& l2) { return inverse(l1)*l2; }

  // Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp
  template<class T>
  inline Vector logmap(const T& l0, const T& lp) { return logmap(between(l0,lp)); }

  /* Exponential map centered at l0, s.t. exp(t,d) = t*exp(d) */
  template<class T>
  inline T expmap(const T& t, const Vector& d) { return t * expmap<T>(d); }

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
    return obj1.equals(obj2);
  }

  // The rest of the file makes double and Vector behave as a Lie type (with + as compose)

  // double,+ group operations
  inline double compose(double p1,double p2) { return p1+p2;}
  inline double inverse(double p) { return -p;}
  inline double between(double p1,double p2) { return p2-p1;}

  // double,+ is a trivial Lie group
  template<> inline double expmap(const Vector& d) { return d(0);}
  template<> inline double expmap(const double& p,const Vector& d) { return p+d(0);}
  inline Vector logmap(const double& p) { return repeat(1,p);}
  inline Vector logmap(const double& p1,const double& p2) { return Vector_(1,p2-p1);}

  // Global functions needed for double
  inline size_t dim(const double& v) { return 1; }

  // Vector group operations
  inline Vector compose(const Vector& p1,const Vector& p2) { return p1+p2;}
  inline Vector inverse(const Vector& p) { return -p;}
  inline Vector between(const Vector& p1,const Vector& p2) { return p2-p1;}

  // Vector is a trivial Lie group
  template<> inline Vector expmap(const Vector& d) { return d;}
  template<> inline Vector expmap(const Vector& p,const Vector& d) { return p+d;}
  inline Vector logmap(const Vector& p) { return p;}
  inline Vector logmap(const Vector& p1,const Vector& p2) { return p2-p1;}

} // namespace gtsam
