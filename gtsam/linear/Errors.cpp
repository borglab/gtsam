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

#include <boost/foreach.hpp>
#include <gtsam/linear/Errors.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Errors::Errors(){}

/* ************************************************************************* */
Errors::Errors(const VectorValues &V) {
	this->resize(V.size()) ;
	int i = 0 ;
	BOOST_FOREACH( Vector &e, *this) {
		e = V[i++];
	}
}

/* ************************************************************************* */
void Errors::print(const std::string& s) const {
  odprintf("%s:\n", s.c_str());
  BOOST_FOREACH(const Vector& v, *this)
    gtsam::print(v);
}

/* ************************************************************************* */
struct equalsVector : public std::binary_function<const Vector&, const Vector&, bool> {
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
    BOOST_FOREACH(const Vector& ai, *this)
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
  BOOST_FOREACH(const Vector& ai, *this)
		result.push_back(ai - *(it++));
	return result;
}

/* ************************************************************************* */
Errors Errors::operator-() const {
  Errors result;
  BOOST_FOREACH(const Vector& ai, *this)
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
  BOOST_FOREACH(const Vector& ai, a)
		result += gtsam::dot(ai, *(it++));
	return result;
}

/* ************************************************************************* */
template<>
void axpy<Errors,Errors>(double alpha, const Errors& x, Errors& y) {
	Errors::const_iterator it = x.begin();
  BOOST_FOREACH(Vector& yi, y)
		axpy(alpha,*(it++),yi);
}

/* ************************************************************************* */
void print(const Errors& a, const string& s) {
	a.print(s);
}

/* ************************************************************************* */

} // gtsam
