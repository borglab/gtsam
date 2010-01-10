/**
 * @file    Vector.h
 * @brief   typedef and functions to augment Boost's ublas::vector<double>
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/random/linear_congruential.hpp>

// Vector is a *global* typedef
// wrap-matlab does this typedef as well
#if ! defined (MEX_H)
typedef boost::numeric::ublas::vector<double> Vector;
#endif

#include "Lie.h"

namespace gtsam {

/**
 * An auxiliary function to printf for Win32 compatibility, added by Kai
 */
void odprintf(const char *format, ...);

/**
 *  constructor with size and initial data, row order !
 */
Vector Vector_( size_t m, const double* const data);

/**
 *  nice constructor, dangerous as number of arguments must be exactly right
 *  and you have to pass doubles !!! always use 0.0 never 0
 */
Vector Vector_(size_t m, ...);

/**
 * Create vector initialized to a constant value
 * @param size
 * @param constant value
 */
Vector repeat(size_t n, double value);

/**
 * Create basis vector of dimension n,
 * with a constant in spot i
 * @param n is the size of the vector
 * @param index of the one
 * @param value is the value to insert into the vector
 * @return delta vector
 */
Vector delta(size_t n, size_t i, double value);

/**
 * Create basis vector of dimension n,
 * with one in spot i
 * @param n is the size of the vector
 * @param index of the one
 * @return basis vector
 */
inline Vector basis(size_t n, size_t i) { return delta(n, i, 1.0); }

/**
 * Create zero vector
 * @param size
 */
inline Vector zero(size_t n) { return repeat(n,0.0);}

/**
 * Create vector initialized to ones
 * @param size
 */
inline Vector ones(size_t n) { return repeat(n,1.0);}
	
/**
 * check if all zero
 */
bool zero(const Vector& v);

/**
 * dimensionality == size
 */
inline size_t dim(const Vector& v) { return v.size(); }
	
/**
 * print with optional string
 */
void print(const Vector& v, const std::string& s = "");

/**
 * operator==()
 */
bool operator==(const Vector& vec1,const Vector& vec2);

/**
 * Greater than or equal to operation
 * returns true if all elements in v1
 * are greater than corresponding elements in v2
 */
bool greaterThanOrEqual(const Vector& v1, const Vector& v2);

/**
 * VecA == VecB up to tolerance
 */
bool equal_with_abs_tol(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * Override of equal in Lie.h
 */
inline bool equal(const Vector& vec1, const Vector& vec2, double tol) {
  return equal_with_abs_tol(vec1, vec2, tol);
}

/**
 * Override of equal in Lie.h
 */
inline bool equal(const Vector& vec1, const Vector& vec2) {
  return equal_with_abs_tol(vec1, vec2);
}

/**
 * Same, prints if error
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
bool assert_equal(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * extract subvector, slice semantics, i.e. range = [i1,i2[ excluding i2
 * @param v Vector
 * @param i1 first row index
 * @param i2 last  row index + 1
 * @return subvector v(i1:i2)
 */
Vector sub(const Vector &v, size_t i1, size_t i2);

/**
 * elementwise multiplication
 * @param a first vector
 * @param b second vector
 * @return vector [a(i)*b(i)]
 */
Vector emul(const Vector &a, const Vector &b);

/**
 * elementwise division
 * @param a first vector
 * @param b second vector
 * @return vector [a(i)/b(i)]
 */
Vector ediv(const Vector &a, const Vector &b);

/**
 * elementwise division, but 0/0 = 0, not inf
 * @param a first vector
 * @param b second vector
 * @return vector [a(i)/b(i)]
 */
Vector ediv_(const Vector &a, const Vector &b);

/**
 * sum vector elements
 * @param a vector
 * @return sum_i a(i)
 */
double sum(const Vector &a);

/** Dot product */
inline double dot(const Vector &a, const Vector& b) { return sum(emul(a,b)); }

/**
 * house(x,j) computes HouseHolder vector v and scaling factor beta
 *  from x, such that the corresponding Householder reflection zeroes out
 *  all but x.(j), j is base 0. Golub & Van Loan p 210.
 */
std::pair<double,Vector> house(Vector &x);

/**
 * Weighted Householder solution vector,
 * a.k.a., the pseudoinverse of the column
 * NOTE: if any sigmas are zero (indicating a constraint)
 * the pseudoinverse will be a selection vector, and the
 * precision will be infinite
 * @param v is the first column of the matrix to solve
 * @param simgas is a vector of standard deviations
 * @return a pair of the pseudoinverse of v and the precision
 */
std::pair<Vector, double> weightedPseudoinverse(const Vector& v, const Vector& sigmas);

/**
 * concatenate Vectors
 */
Vector concatVectors(const std::list<Vector>& vs);

/**
 * concatenate Vectors
 */
Vector concatVectors(size_t nrVectors, ...);

/**
 * random vector
 */
Vector rand_vector_norm(size_t dim, double mean = 0, double sigma = 1);

} // namespace gtsam

static boost::minstd_rand generator(42u);

