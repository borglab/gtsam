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
 */

#include <cstdarg>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <cstdio>

#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>

//#ifdef WIN32
//#include <Windows.h>
//#endif

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void odprintf_(const char *format, ostream& stream, ...) {
	char buf[4096], *p = buf;

	va_list args;
	va_start(args, stream);
#ifdef WIN32
	_vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
#else
	vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
#endif
	va_end(args);

//#ifdef WIN32
//	OutputDebugString(buf);
//#else
	stream << buf;
//#endif
}

/* ************************************************************************* */

void odprintf(const char *format, ...) {
	char buf[4096], *p = buf;

	va_list args;
	va_start(args, format);
#ifdef WIN32
	_vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
#else
	vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
#endif
	va_end(args);

//#ifdef WIN32
//	OutputDebugString(buf);
//#else
	cout << buf;
//#endif
}

/* ************************************************************************* */
Vector Vector_( size_t m, const double* const data) {
  Vector A(m);
  copy(data, data+m, A.data());
  return A;
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
Vector Vector_(const std::vector<double>& d) {
	Vector v(d.size());
	copy(d.begin(), d.end(), v.data());
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
	return Vector::Constant(n, value);
}

/* ************************************************************************* */
Vector delta(size_t n, size_t i, double value) {
	return Vector::Unit(n, i) * value;
}

/* ************************************************************************* */
void print(const Vector& v, const string& s, ostream& stream) {
	size_t n = v.size();
	odprintf_("%s [", stream, s.c_str());
	for(size_t i=0; i<n; i++)
		odprintf_("%g%s", stream, v[i], (i<n-1 ? "; " : ""));
	odprintf_("];\n", stream);
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
		if(isnan(vec1[i]) ^ isnan(vec2[i]))
			return false;
		if(fabs(vec1[i] - vec2[i]) > tol)
			return false;
	}
	return true;
}

/* ************************************************************************* */
bool equal_with_abs_tol(const SubVector& vec1, const SubVector& vec2, double tol) {
	if (vec1.size()!=vec2.size()) return false;
	size_t m = vec1.size();
	for(size_t i=0; i<m; ++i) {
		if(isnan(vec1[i]) ^ isnan(vec2[i]))
			return false;
		if(fabs(vec1[i] - vec2[i]) > tol)
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
	print(expected, "expected");
	print(actual, "actual");
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
	bool flag = false; 	double scale = 1.0;
	size_t m = vec1.size();
	for(size_t i=0; i<m; i++) {
		if((fabs(vec1[i])>tol&&fabs(vec2[i])<tol) || (fabs(vec1[i])<tol&&fabs(vec2[i])>tol))
			return false;
		if(vec1[i] == 0 && vec2[i] == 0) continue;
		if (!flag) {
			scale = vec1[i] / vec2[i];
			flag = true ;
		}
		else if (fabs(vec1[i] - vec2[i]*scale) > tol) return false;
	}
	return flag;
}

/* ************************************************************************* */
ConstSubVector sub(const Vector &v, size_t i1, size_t i2) {
	return v.segment(i1,i2-i1);
}

/* ************************************************************************* */
void subInsert(Vector& fullVector, const Vector& subVector, size_t i) {
	fullVector.segment(i, subVector.size()) = subVector;
}

/* ************************************************************************* */
Vector emul(const Vector &a, const Vector &b) {
	assert (b.size()==a.size());
	return a.cwiseProduct(b);
}

/* ************************************************************************* */
Vector ediv(const Vector &a, const Vector &b) {
	assert (b.size()==a.size());
	return a.cwiseQuotient(b);
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
double sum(const Vector &a) {
	return a.sum();
}

/* ************************************************************************* */
double norm_2(const Vector& v) {
	return v.norm();
}

/* ************************************************************************* */
Vector reciprocal(const Vector &a) {
	size_t n = a.size();
	Vector b(n);
	for( size_t i = 0; i < n; i++ )
		b(i) = 1.0/a(i);
	return b;
}

/* ************************************************************************* */
Vector esqrt(const Vector& v) {
	return v.cwiseSqrt();
}

/* ************************************************************************* */
Vector abs(const Vector& v) {
	return v.cwiseAbs();
}

/* ************************************************************************* */
double max(const Vector &a) {
	return a.maxCoeff();
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
	for (size_t i = 0; i < m; ++i) isZero.push_back(fabs(a[i]) < 1e-9);

	// If there is a valid (a!=0) constraint (sigma==0) return the first one
	for (size_t i = 0; i < m; ++i)
		if (weights[i] == inf && !isZero[i]) {
			pseudo = delta(m, i, 1 / a[i]);
			return inf;
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
	int m = weights.size();
	if (a.size() != m)
		throw invalid_argument("a and weights have different sizes!");
	Vector pseudo(m);
	double precision = weightedPseudoinverse(a, weights, pseudo);
	return make_pair(pseudo, precision);
}

/* ************************************************************************* */
Vector concatVectors(const std::list<Vector>& vs)
{
	size_t dim = 0;
	BOOST_FOREACH(Vector v, vs)
	dim += v.size();

	Vector A(dim);
	size_t index = 0;
	BOOST_FOREACH(Vector v, vs) {
		for(int d = 0; d < v.size(); d++)
			A(d+index) = v(d);
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

/* ************************************************************************* */

} // namespace gtsam
