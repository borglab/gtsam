/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Vector.cpp
 * @brief   typedef and functions to augment Eigen's Vectors
 * @author  Kai Ni
 * @author  Frank Dellaert
 * @author  Varun Agrawal
 */

#include <gtsam/base/Vector.h>
#include <stdexcept>
#include <cstdarg>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace std;

namespace gtsam {

/* *************************************************************************
 * References:
 * 1. https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * 2. https://floating-point-gui.de/errors/comparison/
 * ************************************************************************* */
bool fpEqual(double a, double b, double tol, bool check_relative_also) {
  using std::abs;
  using std::isnan;
  using std::isinf;

  double DOUBLE_MIN_NORMAL = numeric_limits<double>::min() + 1.0;
  double larger = (abs(b) > abs(a)) ? abs(b) : abs(a);

  // handle NaNs
  if(isnan(a) || isnan(b)) {
    return isnan(a) && isnan(b);
  }
  // handle inf
  else if(isinf(a) || isinf(b)) {
    return isinf(a) && isinf(b);
  }
  // If the two values are zero or both are extremely close to it
  // relative error is less meaningful here
  else if(a == 0 || b == 0 || (abs(a) + abs(b)) < DOUBLE_MIN_NORMAL) {
    return abs(a-b) <= tol * DOUBLE_MIN_NORMAL;
  }
  // Check if the numbers are really close.
  // Needed when comparing numbers near zero or tol is in vicinity.
  else if (abs(a - b) <= tol) {
    return true;
  }
  // Check for relative error
  else if (abs(a - b) <=
               tol * min(larger, std::numeric_limits<double>::max()) &&
           check_relative_also) {
    return true;
  }

  return false;
}

/* ************************************************************************* */
//3 argument call
void print(const Vector& v, const string& s, ostream& stream) {
  size_t n = v.size();

  stream << s << "[";
  for(size_t i=0; i<n; i++) {
      stream << setprecision(9) << v(i) << (i<n-1 ? "; " : "");
  }
  stream << "];" << endl;
}

/* ************************************************************************* */
//1 or 2 argument call
void print(const Vector& v, const string& s) {
    print(v, s, cout);
}

/* ************************************************************************* */
void save(const Vector& v, const string &s, const string& filename) {
  fstream stream(filename.c_str(), fstream::out | fstream::app);
  print(v, s + "=", stream);
  stream.close();
}

/* ************************************************************************* */
bool operator==(const Vector& vec1,const Vector& vec2) {
  if (vec1.size() != vec2.size()) return false;
  size_t m = vec1.size();
  for(size_t i=0; i<m; i++)
    if(vec1[i] != vec2[i])
      return false;
  return true;
}

/* ************************************************************************* */
bool greaterThanOrEqual(const Vector& vec1, const Vector& vec2) {
  size_t m = vec1.size();
  for(size_t i=0; i<m; i++)
    if(!(vec1[i] >= vec2[i]))
      return false;
  return true;
}

/* ************************************************************************* */
bool equal_with_abs_tol(const Vector& vec1, const Vector& vec2, double tol) {
  if (vec1.size()!=vec2.size()) return false;
  size_t m = vec1.size();
  for(size_t i=0; i<m; ++i) {
    if (!fpEqual(vec1[i], vec2[i], tol))
      return false;
  }
  return true;
}

/* ************************************************************************* */
bool equal_with_abs_tol(const SubVector& vec1, const SubVector& vec2, double tol) {
  if (vec1.size()!=vec2.size()) return false;
  size_t m = vec1.size();
  for(size_t i=0; i<m; ++i) {
    if (!fpEqual(vec1[i], vec2[i], tol))
      return false;
  }
  return true;
}

/* ************************************************************************* */
bool assert_equal(const Vector& expected, const Vector& actual, double tol) {
  if (equal_with_abs_tol(expected,actual,tol)) return true;
  cout << "not equal:" << endl;
  print(expected, "expected");
  print(actual, "actual");
  return false;
}

/* ************************************************************************* */
bool assert_inequal(const Vector& expected, const Vector& actual, double tol) {
  if (!equal_with_abs_tol(expected,actual,tol)) return true;
  cout << "Erroneously equal:" << endl;
  print(expected, "expected");
  print(actual, "actual");
  return false;
}

/* ************************************************************************* */
bool assert_equal(const SubVector& expected, const SubVector& actual, double tol) {
  if (equal_with_abs_tol(expected,actual,tol)) return true;
  cout << "not equal:" << endl;
  print(static_cast<Vector>(expected), "expected");
  print(static_cast<Vector>(actual), "actual");
  return false;
}

/* ************************************************************************* */
bool assert_equal(const ConstSubVector& expected, const ConstSubVector& actual, double tol) {
  if (equal_with_abs_tol(Vector(expected),Vector(actual),tol)) return true;
  cout << "not equal:" << endl;
  print(Vector(expected), "expected");
  print(Vector(actual), "actual");
  return false;
}

/* ************************************************************************* */
bool linear_dependent(const Vector& vec1, const Vector& vec2, double tol) {
  if (vec1.size()!=vec2.size()) return false;
  bool flag = false;   double scale = 1.0;
  size_t m = vec1.size();
  for(size_t i=0; i<m; i++) {
    if((std::abs(vec1[i])>tol && std::abs(vec2[i])<tol) || (std::abs(vec1[i])<tol && std::abs(vec2[i])>tol))
      return false;
    if(vec1[i] == 0 && vec2[i] == 0) continue;
    if (!flag) {
      scale = vec1[i] / vec2[i];
      flag = true ;
    }
    else if (std::abs(vec1[i] - vec2[i]*scale) > tol) return false;
  }
  return flag;
}

/* ************************************************************************* */
Vector ediv_(const Vector &a, const Vector &b) {
  size_t n = a.size();
  assert (b.size()==a.size());
  Vector c(n);
  for( size_t i = 0; i < n; i++ ) {
    const double &ai = a(i), &bi = b(i);
    c(i) = (bi==0.0 && ai==0.0) ? 0.0 : ai/bi;
  }
  return c;
}

/* ************************************************************************* */
// imperative version, pass in x
double houseInPlace(Vector &v) {
  const double x0 = v(0);
  const double x02 = x0*x0;

  // old code - GSL verison was actually a bit slower
  const double sigma = v.squaredNorm() - x02;

  v(0) = 1.0;

  if( sigma == 0.0 )
    return 0.0;
  else {
    double mu = sqrt(x02 + sigma);
    if( x0 <= 0.0 )
      v(0) = x0 - mu;
    else
      v(0) = -sigma / (x0 + mu);

    const double v02 = v(0)*v(0);
    v = v / v(0);
    return 2.0 * v02 / (sigma + v02);
  }
}

/* ************************************************************************* */
pair<double, Vector > house(const Vector &x) {
  Vector v(x);
  double beta = houseInPlace(v);
  return make_pair(beta, v);
}

/* ************************************************************************* */
// Fast version *no error checking* !
// Pass in initialized vector of size m or will crash !
double weightedPseudoinverse(const Vector& a, const Vector& weights,
    Vector& pseudo) {

  size_t m = weights.size();
  static const double inf = std::numeric_limits<double>::infinity();

  // Check once for zero entries of a. TODO: really needed ?
  vector<bool> isZero;
  for (size_t i = 0; i < m; ++i) isZero.push_back(std::abs(a[i]) < 1e-9);

  // If there is a valid (a!=0) constraint (sigma==0) return the first one
  for (size_t i = 0; i < m; ++i) {
    if (weights[i] == inf && !isZero[i]) {
      // Basically, instead of doing a normal QR step with the weighted
      // pseudoinverse, we enforce the constraint by turning
      // ax + AS = b into x + (A/a)S = b/a, for the first row where a!=0
      pseudo = Vector::Unit(m,i)*(1.0/a[i]);
      return inf;
    }
  }

  // Form psuedo-inverse inv(a'inv(Sigma)a)a'inv(Sigma)
  // For diagonal Sigma, inv(Sigma) = diag(precisions)
  double precision = 0;
  for (size_t i = 0; i < m; i++) {
    double ai = a[i];
    if (!isZero[i]) // also catches remaining sigma==0 rows
      precision += weights[i] * (ai * ai);
  }

  // precision = a'inv(Sigma)a
  if (precision < 1e-9)
    for (size_t i = 0; i < m; i++)
      pseudo[i] = 0;
  else {
    // emul(precisions,a)/precision
    double variance = 1.0 / precision;
    for (size_t i = 0; i < m; i++)
      pseudo[i] = isZero[i] ? 0 : variance * weights[i] * a[i];
  }
  return precision; // sum of weights
}

/* ************************************************************************* */
// Slow version with error checking
pair<Vector, double>
weightedPseudoinverse(const Vector& a, const Vector& weights) {
  DenseIndex m = weights.size();
  if (a.size() != m)
    throw invalid_argument("a and weights have different sizes!");
  Vector pseudo(m);
  double precision = weightedPseudoinverse(a, weights, pseudo);
  return make_pair(pseudo, precision);
}

/* ************************************************************************* */
Vector concatVectors(const std::list<Vector>& vs) {
  size_t dim = 0;
  for (const Vector& v : vs) dim += v.size();

  Vector A(dim);
  size_t index = 0;
  for (const Vector& v : vs) {
    for (int d = 0; d < v.size(); d++) A(d + index) = v(d);
    index += v.size();
  }

  return A;
}

/* ************************************************************************* */
Vector concatVectors(size_t nrVectors, ...)
{
  va_list ap;
  list<Vector> vs;
  va_start(ap, nrVectors);
  for( size_t i = 0 ; i < nrVectors ; i++) {
    Vector* V = va_arg(ap, Vector*);
    vs.push_back(*V);
  }
  va_end(ap);
  return concatVectors(vs);
}

} // namespace gtsam
