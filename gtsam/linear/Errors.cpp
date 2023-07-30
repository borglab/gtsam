/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Errors.cpp
 * @brief   Factor Graph Values
 * @brief   Errors
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <gtsam/linear/Errors.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Errors createErrors(const VectorValues& V) {
  Errors result;
  for (const auto& [key, e] : V) {
    result.push_back(e);
  }
  return result;
}

/* ************************************************************************* */
void print(const Errors& e, const string& s) {
  cout << s << endl;
  for(const Vector& v: e)
    gtsam::print(v);
}

/* ************************************************************************* */
struct equalsVector : public std::function<bool(const Vector&, const Vector&)> {
  double tol_;
  equalsVector(double tol = 1e-9) : tol_(tol) {}
  bool operator()(const Vector& expected, const Vector& actual) {
    return equal_with_abs_tol(expected, actual,tol_);
  }
};

bool equality(const Errors& actual, const Errors& expected, double tol) {
  if (actual.size() != expected.size()) return false;
  return equal(actual.begin(), actual.end(), expected.begin(),
               equalsVector(tol));
}

/* ************************************************************************* */
Errors operator+(const Errors& a, const Errors& b) {
#ifndef NDEBUG
  size_t m = a.size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::operator+: incompatible sizes"));
#endif
  Errors result;
  Errors::const_iterator it = b.begin();
    for(const Vector& ai: a)
    result.push_back(ai + *(it++));
  return result;
}


/* ************************************************************************* */
Errors operator-(const Errors& a, const Errors& b) {
#ifndef NDEBUG
  size_t m = a.size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::operator-: incompatible sizes"));
#endif
  Errors result;
  Errors::const_iterator it = b.begin();
  for(const Vector& ai: a)
    result.push_back(ai - *(it++));
  return result;
}

/* ************************************************************************* */
Errors operator-(const Errors& a) {
  Errors result;
  for(const Vector& ai: a)
    result.push_back(-ai);
  return result;
}

/* ************************************************************************* */
double dot(const Errors& a, const Errors& b) {
#ifndef NDEBUG
  size_t m = a.size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::dot: incompatible sizes"));
#endif
  double result = 0.0;
  Errors::const_iterator it = b.begin();
  for(const Vector& ai: a)
    result += gtsam::dot<Vector,Vector>(ai, *(it++));
  return result;
}

/* ************************************************************************* */
void axpy(double alpha, const Errors& x, Errors& y) {
  Errors::const_iterator it = x.begin();
  for(Vector& yi: y)
    yi += alpha * (*(it++));
}

/* ************************************************************************* */

} // gtsam
