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

#include <boost/range/adaptor/map.hpp>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Errors::Errors(){}

/* ************************************************************************* */
Errors::Errors(const VectorValues& V) {
  for(const Vector& e: V | boost::adaptors::map_values) {
    push_back(e);
  }
}

/* ************************************************************************* */
void Errors::print(const std::string& s) const {
  cout << s << endl;
  for(const Vector& v: *this)
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

bool Errors::equals(const Errors& expected, double tol) const {
  if( size() != expected.size() ) return false;
  return equal(begin(),end(),expected.begin(),equalsVector(tol));
}

/* ************************************************************************* */
Errors Errors::operator+(const Errors& b) const {
#ifndef NDEBUG
  size_t m = size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::operator+: incompatible sizes"));
#endif
  Errors result;
  Errors::const_iterator it = b.begin();
    for(const Vector& ai: *this)
    result.push_back(ai + *(it++));
  return result;
}


/* ************************************************************************* */
Errors Errors::operator-(const Errors& b) const {
#ifndef NDEBUG
  size_t m = size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::operator-: incompatible sizes"));
#endif
  Errors result;
  Errors::const_iterator it = b.begin();
  for(const Vector& ai: *this)
    result.push_back(ai - *(it++));
  return result;
}

/* ************************************************************************* */
Errors Errors::operator-() const {
  Errors result;
  for(const Vector& ai: *this)
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
    result += gtsam::dot(ai, *(it++));
  return result;
}

/* ************************************************************************* */
template<>
void axpy(double alpha, const Errors& x, Errors& y) {
  Errors::const_iterator it = x.begin();
  for(Vector& yi: y)
    yi += alpha * (*(it++));
}

/* ************************************************************************* */
void print(const Errors& a, const string& s) {
  a.print(s);
}

/* ************************************************************************* */

} // gtsam
