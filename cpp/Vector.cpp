/**
* @file    Vector.cpp
* @brief   typedef and functions to augment Boost's ublas::vector<double>
* @author  Kai Ni
* @author  Frank Dellaert
*/

#include <stdarg.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <boost/foreach.hpp>
#include <stdio.h>

#ifdef WIN32
#include <Windows.h>
#endif

#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "Vector.h"

using namespace std;

namespace gtsam {
  
  void odprintf(const char *format, ...)
  {
    char    buf[4096], *p = buf;
    int     n;
    
    va_list args;
    va_start(args, format);
    #ifdef WIN32
    n = _vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
    #else
    n = vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
    #endif
    va_end(args);
    
    /*
    p += (n < 0) ? sizeof buf - 3 : n;
    
    while ( p > buf  &&  isspace(p[-1]) )
      *--p = '\0';
    
    *p++ = '\r';
    *p++ = '\n';
    *p   = '\0';
    */
    
    #ifdef WIN32
    OutputDebugString(buf);
    #else    
    printf(buf);
    #endif
    
  }
  
  /* ************************************************************************* */
  Vector Vector_( size_t m, const double* const data) {
    Vector v(m);
    copy(data, data+m, v.data().begin());
    return v;
  }
  
  /* ************************************************************************* */
  Vector Vector_(size_t m, ...) {
    Vector v(m);
    va_list ap;
    va_start(ap, m);
    for( size_t i = 0 ; i < m ; i++) {
      double value = va_arg(ap, double);
      v(i) = value;
    }
    va_end(ap);
    return v;
  }
  
  /* ************************************************************************* */
  bool zero(const Vector& v) {
    bool result = true;
    size_t n = v.size();
    for( size_t j = 0 ; j < n ; j++)
      result = result && (v(j) == 0.0);
    return result;
  }
  
  /* ************************************************************************* */
  Vector repeat(size_t n, double value) {
    Vector v(n); fill_n(v.begin(),n,value);
    return v;
  }

  /* ************************************************************************* */
  Vector delta(size_t n, size_t i, double value) {
	  Vector v = zero(n);
	  v(i) = value;
	  return v;
  }

  /* ************************************************************************* */
  void print(const Vector& v, const string& s) {
    size_t n = v.size();
    odprintf("%s[", s.c_str());
    for(size_t i=0; i<n; i++)
      odprintf("%g%s", v[i], (i<n-1 ? "; " : ""));
    odprintf("]\n");
  }
  
  /* ************************************************************************* */
  bool operator==(const Vector& vec1,const Vector& vec2) {
    Vector::const_iterator it1 = vec1.begin();
    Vector::const_iterator it2 = vec2.begin();
    size_t m = vec1.size();
    for(size_t i=0; i<m; i++)
      if(it1[i] != it2[i])
      return false;
    return true;
  }
  
  /* ************************************************************************* */
  bool equal_with_abs_tol(const Vector& vec1, const Vector& vec2, double tol) {
    Vector::const_iterator it1 = vec1.begin();
    Vector::const_iterator it2 = vec2.begin();
    if (vec1.size()!=vec2.size()) return false;
    for(size_t i=0; i<vec1.size(); i++) {
    	if(isnan(it1[i]) xor isnan(it2[i]))
    		return false;
    	if(fabs(it1[i] - it2[i]) > tol)
    		return false;
    }
    return true;
  }
  
  /* ************************************************************************* */
  bool assert_equal(const Vector& vec1, const Vector& vec2, double tol) {
    if (equal_with_abs_tol(vec1,vec2,tol)) return true;
    cout << "not equal:" << endl;
    print(vec1, "v1");
    print(vec2, "v2");
    return false;
  }
  
  /* ************************************************************************* */
  Vector sub(const Vector &v, size_t i1, size_t i2) {
    size_t n = i2-i1;
    Vector v_return(n);
    for( size_t i = 0; i < n; i++ )
      v_return(i) = v(i1 + i);
    return v_return;
  }
  
  /* ************************************************************************* */
  Vector ediv(const Vector &a, const Vector &b) {
  	size_t n = a.size();
		assert (b.size()==n);
		Vector c(n);
		for( size_t i = 0; i < n; i++ )
			c(i) = a(i)/b(i);
		return c;
		}

  /* ************************************************************************* */
  pair<double, Vector > house(Vector &x)
  {
    const double x02 = x(0)*x(0);
    const double sigma = inner_prod(trans(x),x) - x02;
    double beta = 0.0;
    
    Vector v(x); v(0) = 1.0;
    
    if( sigma == 0.0 )
      beta = 0.0;
    else {
      double mu = sqrt(x02 + sigma);
      if( x(0) <= 0.0 )
        v(0) = x(0) - mu;
      else
        v(0) = -sigma / (x(0) + mu);
      
      const double v02 = v(0)*v(0);
      beta = 2.0 * v02 / (sigma + v02);
      v = v / v(0);
    }
    
    return make_pair(beta, v);
  }
  
  /* ************************************************************************* */
  pair<Vector, double> weightedPseudoinverse(const Vector& a, const Vector& sigmas) {
	  size_t m = sigmas.size();
	  if (a.size() != m)
		  throw invalid_argument("V and precisions have different sizes!");

	  // detect constraints and sanity-check
	  for(int i=0; i<m; ++i)
		  if (sigmas[i] < 1e-9 && fabs(a[i]) > 1e-9)
			  return make_pair(delta(m,i,1/a[i]), 1.0/0.0);

	  // normal case
	  double normV = 0.;
	  Vector precisions(m);
	  for(int i = 0; i<a.size(); i++) {
		  if (sigmas[i] < 1e-5) {
			  precisions[i] = 1./0.;
		  } else {
			  precisions[i] = 1./(sigmas[i]*sigmas[i]);
			  normV += a[i]*a[i]*precisions[i];
		  }
	  }
	  Vector sol = zero(a.size());
	  for(int i = 0; i<a.size(); i++)
		  if (sigmas[i] > 1e-5)
			  sol[i] = precisions[i]*a[i];
	  return make_pair(sol/normV, normV);
  }

  /* ************************************************************************* */
  Vector concatVectors(size_t nrVectors, ...)
  {
    int dimA = 0;
    va_list ap;
    va_start(ap, nrVectors);
    Vector* V;
    for( size_t i = 0 ; i < nrVectors ; i++)
    {
      V = va_arg(ap, Vector*);
      dimA += V->size();
    }
    va_end(ap);
    va_start(ap, nrVectors);
    Vector A(dimA);
    int index = 0;
    for( size_t i = 0 ; i < nrVectors ; i++)
    {
      V = va_arg(ap, Vector *);
      for(size_t d = 0; d < V->size(); d++)
        A(d+index) = (*V)(d);
      index += V->size();
    }  
    
    return A;
  }
  
  /* ************************************************************************* */
  Vector rand_vector_norm(size_t dim, double mean, double sigma)
  {
    boost::normal_distribution<double> norm_dist(mean, sigma);
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<double> > norm(generator, norm_dist);
    
    Vector v(dim);
    Vector::iterator it_v;
    for(it_v=v.begin(); it_v!=v.end(); it_v++)
      *it_v = norm();
    
    return v;
  }
  
  /* ************************************************************************* */
  
  
  
} // namespace gtsam
