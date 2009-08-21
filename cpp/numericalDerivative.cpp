/**
 * @file    numericalDerivative.cpp
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

#include "numericalDerivative.h"

namespace gtsam {

/* ************************************************************************* */
  Matrix NumericalDerivative11
  (Vector (*h)(const Vector&), const Vector& x_, double delta) {
    Vector x(x_), hx = h(x);
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x.size();
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      x(j) +=   delta; Vector hxplus = h(x);
      x(j) -= 2*delta; Vector hxmin  = h(x);
      x(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

/* ************************************************************************* */
  Matrix NumericalDerivative21
  (Vector (*h)(const Vector&, const Vector&), const Vector& x1_, const Vector& x2, double delta) {
    Vector x1(x1_), hx = h(x1,x2);
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x1.size();
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      x1(j) +=   delta; Vector hxplus = h(x1,x2);
      x1(j) -= 2*delta; Vector hxmin  = h(x1,x2);
      x1(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }

/* ************************************************************************* */
  Matrix NumericalDerivative22
  (Vector (*h)(const Vector&, const Vector&), const Vector& x1, const Vector& x2_, double delta) {
    Vector x2(x2_), hx = h(x1,x2);
    double factor = 1.0/(2.0*delta);
    const size_t m = hx.size(), n = x2.size();
    Matrix H = zeros(m,n);
    for (size_t j=0;j<n;j++) {
      x2(j) +=   delta; Vector hxplus = h(x1,x2);
      x2(j) -= 2*delta; Vector hxmin  = h(x1,x2);
      x2(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
      for (size_t i=0;i<m;i++) H(i,j) = dh(i);
    }
    return H;
  }
/* ************************************************************************* */

}
