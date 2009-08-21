/**
 * @file    numericalDerivative.h
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Matrix.h"

namespace gtsam {

  /**
   * Compute numerical derivative in arument 1 of unary function
   * @param h unary function yielding m-vector
   * @param x n-dimensional value at which to evaluate h
   * @param delta increment for numerical derivative
   * @return m*n Jacobian computed via central differencing
   */
  Matrix NumericalDerivative11
   (Vector (*h)(const Vector&), const Vector& x, double delta=1e-5);

  /**
	* templated version (starts with LOWERCASE n)
	 * Class Y is the output arguement
	 * Class X is the input argument
   * both classes need dim, exmap, vector
   */
  template<class Y, class X>
    Matrix numericalDerivative11
    (Y (*h)(const X&), const X& x, double delta=1e-5) {
    Vector hx = h(x).vector();
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x.dim();
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      d(j) +=   delta; Vector hxplus = h(x.exmap(d)).vector(); 
      d(j) -= 2*delta; Vector hxmin  = h(x.exmap(d)).vector();
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
   */
  Matrix NumericalDerivative21
    (Vector (*h)(const Vector&, const Vector&), const Vector& x1, const Vector& x2, double delta=1e-5);

  /**
   * templated version (starts with LOWERCASE n)
   * classes need dim, exmap, vector
   */
   template<class Y, class X1, class X2>
    Matrix numericalDerivative21
    (Y (*h)(const X1&, const X2&), 
     const X1& x1, const X2& x2, double delta=1e-5) 
  {
    Vector hx = h(x1,x2).vector();
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x1.dim();
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      d(j) +=   delta; Vector hxplus = h(x1.exmap(d),x2).vector();
      d(j) -= 2*delta; Vector hxmin  = h(x1.exmap(d),x2).vector();
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

  /**
   * Compute numerical derivative in arument 2 of binary function
   * @param h binary function yielding m-vector
   * @param x1 first argument value
   * @param x2 n-dimensional second argument value
   * @param delta increment for numerical derivative
   * @return m*n Jacobian computed via central differencing
   */
  Matrix NumericalDerivative22
    (Vector (*h)(const Vector&, const Vector&), const Vector& x1, const Vector& x2, double delta=1e-5);

  /**
   *  templated version (starts with LOWERCASE n)
   * classes need dim, exmap, vector
   */
  template<class Y, class X1, class X2>
    Matrix numericalDerivative22
    (Y (*h)(const X1&, const X2&), 
     const X1& x1, const X2& x2, double delta=1e-5) 
  {
    Vector hx = h(x1,x2).vector();
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x2.dim();
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      d(j) +=   delta; Vector hxplus = h(x1,x2.exmap(d)).vector();
      d(j) -= 2*delta; Vector hxmin  = h(x1,x2.exmap(d)).vector();
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
   */
  Matrix NumericalDerivative31
    (Vector (*h)(const Vector&, const Vector&, const Vector&), const Vector& x1, const Vector& x2, const Vector& x3, double delta=1e-5);

  /**
   * templated version (starts with LOWERCASE n)
   * classes need dim, exmap, vector
   */
  template<class Y, class X1, class X2, class X3>
    Matrix numericalDerivative31
    (Y (*h)(const X1&, const X2&, const X3&), 
     const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) 
  {
    Vector hx = h(x1,x2,x3).vector();
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x1.dim();
    Vector d(n,0.0);
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      d(j) +=   delta; Vector hxplus = h(x1.exmap(d),x2,x3).vector();
      d(j) -= 2*delta; Vector hxmin  = h(x1.exmap(d),x2,x3).vector();
      d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

}
