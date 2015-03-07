/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Vector.h
 * @brief   typedef and functions to augment Eigen's VectorXd
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <vector>
#include <iostream>
#include <gtsam/base/types.h>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

namespace gtsam {

// Vector is just a typedef of the Eigen dynamic vector type

// Typedef arbitary length vector
typedef Eigen::VectorXd Vector;

// Commonly used fixed size vectors
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix<double, 6, 1> Vector6;

typedef Eigen::VectorBlock<Vector> SubVector;
typedef Eigen::VectorBlock<const Vector> ConstSubVector;

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
 * Create a numeric vector from an STL vector of doubles
 */
Vector Vector_(const std::vector<double>& data);

/**
 * Create vector initialized to a constant value
 * @param n is the size of the vector
 * @param value is a constant value to insert into the vector
 */
Vector repeat(size_t n, double value);

/**
 * Create basis vector of dimension n,
 * with a constant in spot i
 * @param n is the size of the vector
 * @param i index of the one
 * @param value is the value to insert into the vector
 * @return delta vector
 */
Vector delta(size_t n, size_t i, double value);

/**
 * Create basis vector of dimension n,
 * with one in spot i
 * @param n is the size of the vector
 * @param i index of the one
 * @return basis vector
 */
inline Vector basis(size_t n, size_t i) { return delta(n, i, 1.0); }

/**
 * Create zero vector
 * @param n size
 */
inline Vector zero(size_t n) { return Vector::Zero(n);}

/**
 * Create vector initialized to ones
 * @param n size
 */
inline Vector ones(size_t n) { return Vector::Ones(n); }
	
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
void print(const Vector& v, const std::string& s = "", std::ostream& stream = std::cout);

/**
 * save a vector to file, which can be loaded by matlab
 */
void save(const Vector& A, const std::string &s, const std::string& filename);

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
bool equal_with_abs_tol(const SubVector& vec1, const SubVector& vec2, double tol=1e-9);

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
 * Not the same, prints if error
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
bool assert_inequal(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * Same, prints if error
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
bool assert_equal(const SubVector& vec1, const SubVector& vec2, double tol=1e-9);
bool assert_equal(const ConstSubVector& vec1, const ConstSubVector& vec2, double tol=1e-9);

/**
 * check whether two vectors are linearly dependent
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
bool linear_dependent(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * extract subvector, slice semantics, i.e. range = [i1,i2[ excluding i2
 * @param v Vector
 * @param i1 first row index
 * @param i2 last  row index + 1
 * @return subvector v(i1:i2)
 */
ConstSubVector sub(const Vector &v, size_t i1, size_t i2);

/**
 * Inserts a subvector into a vector IN PLACE
 * @param fullVector is the vector to be changed
 * @param subVector is the vector to insert
 * @param i is the index where the subvector should be inserted
 */
void subInsert(Vector& fullVector, const Vector& subVector, size_t i);

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

/**
 * Calculates L2 norm for a vector
 * modeled after boost.ublas for compatibility
 * @param v vector
 * @return the L2 norm
 */
double norm_2(const Vector& v);

/**
 * Elementwise reciprocal of vector elements
 * @param a vector
 * @return [1/a(i)]
 */
Vector reciprocal(const Vector &a);

/**
 * Elementwise sqrt of vector elements
 * @param v is a vector
 * @return [sqrt(a(i))]
 */
Vector esqrt(const Vector& v);

/**
 * Absolute values of vector elements
 * @param v is a vector
 * @return [abs(a(i))]
 */
Vector abs(const Vector& v);

/**
 * Return the max element of a vector
 * @param a is a vector
 * @return max(a)
 */
double max(const Vector &a);

/**
 * Dot product
 */
template<class V1, class V2>
inline double dot(const V1 &a, const V2& b) {
	assert (b.size()==a.size());
	return a.dot(b);
}

/** compatibility version for ublas' inner_prod() */
template<class V1, class V2>
inline double inner_prod(const V1 &a, const V2& b) {
	assert (b.size()==a.size());
	return a.dot(b);
}

/**
 * BLAS Level 1 scal: x <- alpha*x
 * \deprecated: use operators instead
 */
inline void scal(double alpha, Vector& x) { x *= alpha; }

/**
 * BLAS Level 1 axpy: y <- alpha*x + y
 * \deprecated: use operators instead
 */
template<class V1, class V2>
inline void axpy(double alpha, const V1& x, V2& y) {
	assert (y.size()==x.size());
	y += alpha * x;
}
inline void axpy(double alpha, const Vector& x, SubVector y) {
	assert (y.size()==x.size());
	y += alpha * x;
}

/**
 * house(x,j) computes HouseHolder vector v and scaling factor beta
 *  from x, such that the corresponding Householder reflection zeroes out
 *  all but x.(j), j is base 0. Golub & Van Loan p 210.
 */
std::pair<double,Vector> house(const Vector &x);

/** beta = house(x) computes the HouseHolder vector in place */
double houseInPlace(Vector &x);

/**
 * Weighted Householder solution vector,
 * a.k.a., the pseudoinverse of the column
 * NOTE: if any sigmas are zero (indicating a constraint)
 * the pseudoinverse will be a selection vector, and the
 * variance will be zero
 * @param v is the first column of the matrix to solve
 * @param weights is a vector of weights/precisions where w=1/(s*s)
 * @return a pair of the pseudoinverse of v and the associated precision/weight
 */
std::pair<Vector, double>
weightedPseudoinverse(const Vector& v, const Vector& weights);

/*
 * Fast version *no error checking* !
 * Pass in initialized vector pseudo of size(weights) or will crash !
 * @return the precision, pseudoinverse in third argument
 */
double weightedPseudoinverse(const Vector& a, const Vector& weights, Vector& pseudo);

/**
 * concatenate Vectors
 */
Vector concatVectors(const std::list<Vector>& vs);

/**
 * concatenate Vectors
 */
Vector concatVectors(size_t nrVectors, ...);

} // namespace gtsam

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

// split version - copies into an STL vector for serialization
template<class Archive>
void save(Archive & ar, const gtsam::Vector & v, unsigned int version)
{
	const size_t n = v.size();
	std::vector<double> raw_data(n);
	copy(v.data(), v.data()+n, raw_data.begin());
	ar << make_nvp("data", raw_data);
}
template<class Archive>
void load(Archive & ar, gtsam::Vector & v, unsigned int version)
{
	std::vector<double> raw_data;
	ar >> make_nvp("data", raw_data);
	v = gtsam::Vector_(raw_data);
}

} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector)
