/**
 * @file    numericalDerivative.h
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Matrix.h"

//#define LINEARIZE_AT_IDENTITY

namespace gtsam {

  /**
   * Numerically compute gradient of scalar function
   * Class X is the input argument
   * The class X needs to have dim, expmap, logmap
   */
  template<class X>
  Vector numericalGradient(double (*h)(const X&), const X& x, double delta=1e-5) {
    double hx = h(x);
    double factor = 1.0/(2.0*delta);
    const size_t n = x.dim();
    Vector d(n,0.0), g(n,0.0);
    for (size_t j=0;j<n;j++) {
      d(j) +=   delta; double hxplus = h(expmap(x,d));
      d(j) -= 2*delta; double hxmin  = h(expmap(x,d));
      d(j) +=   delta; g(j) = (hxplus-hxmin)*factor;
    }
    return g;
  }

  /**
   * Compute numerical derivative in argument 1 of unary function
   * @param h unary function yielding m-vector
   * @param x n-dimensional value at which to evaluate h
   * @param delta increment for numerical derivative
   * Class Y is the output argument
   * Class X is the input argument
   * @return m*n Jacobian computed via central differencing
   * Both classes X,Y need dim, expmap, logmap
   */
  template<class Y, class X>
  Matrix numericalDerivative11(Y (*h)(const X&), const X& x, double delta=1e-5) {
    Y hx = h(x);
    double factor = 1.0/(2.0*delta);
    const size_t m = dim(hx), n = dim(x);
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
#ifdef LINEARIZE_AT_IDENTITY
      d(j) +=   delta; Vector hxplus = logmap(h(expmap(x,d)));
      d(j) -= 2*delta; Vector hxmin  = logmap(h(expmap(x,d)));
#else
      d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x,d)));
      d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x,d)));
#endif
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

  /**
   * Compute numerical derivative in argument 1 of binary function
   * @param h binary function yielding m-vector
   * @param x1 n-dimensional first argument value
   * @param x2 second argument value
   * @param delta increment for numerical derivative
   * @return m*n Jacobian computed via central differencing
   * All classes Y,X1,X2 need dim, expmap, logmap
   */
  template<class Y, class X1, class X2>
  Matrix numericalDerivative21(Y (*h)(const X1&, const X2&),
      const X1& x1, const X2& x2, double delta=1e-5) {
    Y hx = h(x1,x2);
    double factor = 1.0/(2.0*delta);
    const size_t m = dim(hx), n = dim(x1);
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
#ifdef LINEARIZE_AT_IDENTITY
      d(j) +=   delta; Vector hxplus = logmap(h(expmap(x1,d),x2));
      d(j) -= 2*delta; Vector hxmin  = logmap(h(expmap(x1,d),x2));
#else
      d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x1,d),x2));
      d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x1,d),x2));
#endif
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

  /**
   * Compute numerical derivative in argument 2 of binary function
   * @param h binary function yielding m-vector
   * @param x1 first argument value
   * @param x2 n-dimensional second argument value
   * @param delta increment for numerical derivative
   * @return m*n Jacobian computed via central differencing
   * All classes Y,X1,X2 need dim, expmap, logmap
   */
  template<class Y, class X1, class X2>
  Matrix numericalDerivative22
  (Y (*h)(const X1&, const X2&),
      const X1& x1, const X2& x2, double delta=1e-5)
  {
    Y hx = h(x1,x2);
    double factor = 1.0/(2.0*delta);
    const size_t m = dim(hx), n = dim(x2);
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
#ifdef LINEARIZE_AT_IDENTITY
      d(j) +=   delta; Vector hxplus = logmap(h(x1,expmap(x2,d)));
      d(j) -= 2*delta; Vector hxmin  = logmap(h(x1,expmap(x2,d)));
#else
      d(j) +=   delta; Vector hxplus = logmap(hx, h(x1,expmap(x2,d)));
      d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(x1,expmap(x2,d)));
#endif
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

  /**
   * Compute numerical derivative in argument 1 of binary function
   * @param h binary function yielding m-vector
   * @param x1 n-dimensional first argument value
   * @param x2 second argument value
   * @param delta increment for numerical derivative
   * @return m*n Jacobian computed via central differencing
   * All classes Y,X1,X2,X3 need dim, expmap, logmap
   */
  template<class Y, class X1, class X2, class X3>
  Matrix numericalDerivative31
  (Y (*h)(const X1&, const X2&, const X3&),
      const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
  {
    Y hx = h(x1,x2,x3);
    double factor = 1.0/(2.0*delta);
    const size_t m = dim(hx), n = dim(x1);
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
#ifdef LINEARIZE_AT_IDENTITY
      d(j) +=   delta; Vector hxplus = logmap(h(expmap(x1,d),x2,x3));
      d(j) -= 2*delta; Vector hxmin  = logmap(h(expmap(x1,d),x2,x3));
#else
      d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x1,d),x2,x3));
      d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x1,d),x2,x3));
#endif
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

}
