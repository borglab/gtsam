/*
 * Lie.h
 *
 *  Created on: Jan 5, 2010
 *      Author: Richard Roberts
 */

#ifndef LIE_H_
#define LIE_H_

#include "Vector.h"
#include "VectorConfig.h"
#include "Matrix.h"


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

}


#endif /* LIE_H_ */
