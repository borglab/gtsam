/**
 * @file    Errors.cpp
 * @brief   Factor Graph Configuration
 * @brief   Errors
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include "Errors.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void Errors::print(const std::string& s) const {
  odprintf("%s:\n", s.c_str());
  for (size_t i=0;i<size();i++) {
    odprintf("%d:", i);
    gtsam::print((*this)[i]);
  }
}

/* ************************************************************************* */
bool Errors::equals(const Errors& expected, double tol) const {
  if( size() != expected.size() ) return false;
  for (size_t i=0;i<size();i++)
    if(!equal_with_abs_tol(expected[i],(*this)[i],tol))
    	return false;
  return true;
}

/* ************************************************************************* */
double dot(const Errors& a, const Errors& b) {
	size_t m = a.size();
  if (b.size()!=m)
    throw(std::invalid_argument("Errors::dot: incompatible sizes"));
	double result = 0.0;
	for (size_t i = 0; i < m; i++)
		result += gtsam::dot(a[i], b[i]);
	return result;
}

/* ************************************************************************* */

} // gtsam
