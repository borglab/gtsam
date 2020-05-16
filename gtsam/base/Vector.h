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
 * @author  Alex Hagiopol
 * @author  Varun Agrawal
 */

// \callgraph

#pragma once
#ifndef MKL_BLAS
#define MKL_BLAS MKL_DOMAIN_BLAS
#endif

#include <gtsam/global_includes.h>
#include <Eigen/Core>
#include <iosfwd>
#include <list>

namespace gtsam {

// Vector is just a typedef of the Eigen dynamic vector type

// Typedef arbitary length vector
typedef Eigen::VectorXd Vector;

// Commonly used fixed size vectors
typedef Eigen::Matrix<double, 1, 1> Vector1;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;

static const Eigen::MatrixBase<Vector2>::ConstantReturnType Z_2x1 = Vector2::Zero();
static const Eigen::MatrixBase<Vector3>::ConstantReturnType Z_3x1 = Vector3::Zero();

// Create handy typedefs and constants for vectors with N>3
// VectorN and Z_Nx1, for N=1..9
#define GTSAM_MAKE_VECTOR_DEFS(N)                \
  typedef Eigen::Matrix<double, N, 1> Vector##N; \
  static const Eigen::MatrixBase<Vector##N>::ConstantReturnType Z_##N##x1 = Vector##N::Zero();

GTSAM_MAKE_VECTOR_DEFS(4);
GTSAM_MAKE_VECTOR_DEFS(5);
GTSAM_MAKE_VECTOR_DEFS(6);
GTSAM_MAKE_VECTOR_DEFS(7);
GTSAM_MAKE_VECTOR_DEFS(8);
GTSAM_MAKE_VECTOR_DEFS(9);
GTSAM_MAKE_VECTOR_DEFS(10);
GTSAM_MAKE_VECTOR_DEFS(11);
GTSAM_MAKE_VECTOR_DEFS(12);

typedef Eigen::VectorBlock<Vector> SubVector;
typedef Eigen::VectorBlock<const Vector> ConstSubVector;

/**
  * Ensure we are not including a different version of Eigen in user code than
  * while compiling gtsam, since it can lead to hard-to-understand runtime
  * crashes.
  */
#if defined(GTSAM_EIGEN_VERSION_WORLD)
static_assert(
    GTSAM_EIGEN_VERSION_WORLD==EIGEN_WORLD_VERSION &&
    GTSAM_EIGEN_VERSION_MAJOR==EIGEN_MAJOR_VERSION,
  "Error: GTSAM was built against a different version of Eigen");
#endif

/**
 * Numerically stable function for comparing if floating point values are equal
 * within epsilon tolerance.
 * Used for vector and matrix comparison with C++11 compatible functions.
 *
 * If either value is NaN or Inf, we check for both values to be NaN or Inf
 * respectively for the comparison to be true.
 * If one is NaN/Inf and the other is not, returns false.
 *
 * Return true if two numbers are close wrt tol.
 */
GTSAM_EXPORT bool fpEqual(double a, double b, double tol);

/**
 * print without optional string, must specify cout yourself
 */
GTSAM_EXPORT void print(const Vector& v, const std::string& s, std::ostream& stream);

/**
 * print with optional string to cout
 */
GTSAM_EXPORT void print(const Vector& v, const std::string& s = "");

/**
 * save a vector to file, which can be loaded by matlab
 */
GTSAM_EXPORT void save(const Vector& A, const std::string &s, const std::string& filename);

/**
 * operator==()
 */
GTSAM_EXPORT bool operator==(const Vector& vec1,const Vector& vec2);

/**
 * Greater than or equal to operation
 * returns true if all elements in v1
 * are greater than corresponding elements in v2
 */
GTSAM_EXPORT bool greaterThanOrEqual(const Vector& v1, const Vector& v2);

/**
 * VecA == VecB up to tolerance
 */
GTSAM_EXPORT bool equal_with_abs_tol(const Vector& vec1, const Vector& vec2, double tol=1e-9);
GTSAM_EXPORT bool equal_with_abs_tol(const SubVector& vec1, const SubVector& vec2, double tol=1e-9);

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
GTSAM_EXPORT bool assert_equal(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * Not the same, prints if error
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
GTSAM_EXPORT bool assert_inequal(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * Same, prints if error
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
GTSAM_EXPORT bool assert_equal(const SubVector& vec1, const SubVector& vec2, double tol=1e-9);
GTSAM_EXPORT bool assert_equal(const ConstSubVector& vec1, const ConstSubVector& vec2, double tol=1e-9);

/**
 * check whether two vectors are linearly dependent
 * @param vec1 Vector
 * @param vec2 Vector
 * @param tol 1e-9
 * @return bool
 */
GTSAM_EXPORT bool linear_dependent(const Vector& vec1, const Vector& vec2, double tol=1e-9);

/**
 * elementwise division, but 0/0 = 0, not inf
 * @param a first vector
 * @param b second vector
 * @return vector [a(i)/b(i)]
 */
GTSAM_EXPORT Vector ediv_(const Vector &a, const Vector &b);

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
GTSAM_EXPORT std::pair<double,Vector> house(const Vector &x);

/** beta = house(x) computes the HouseHolder vector in place */
GTSAM_EXPORT double houseInPlace(Vector &x);

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
GTSAM_EXPORT std::pair<Vector, double>
weightedPseudoinverse(const Vector& v, const Vector& weights);

/*
 * Fast version *no error checking* !
 * Pass in initialized vector pseudo of size(weights) or will crash !
 * @return the precision, pseudoinverse in third argument
 */
GTSAM_EXPORT double weightedPseudoinverse(const Vector& a, const Vector& weights, Vector& pseudo);

/**
 * concatenate Vectors
 */
GTSAM_EXPORT Vector concatVectors(const std::list<Vector>& vs);

/**
 * concatenate Vectors
 */
GTSAM_EXPORT Vector concatVectors(size_t nrVectors, ...);

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
inline Vector abs(const Vector& v){return v.cwiseAbs();}
inline Vector basis(size_t n, size_t i) { return Vector::Unit(n,i); }
inline Vector delta(size_t n, size_t i, double value){ return Vector::Unit(n, i) * value;}
inline size_t dim(const Vector& v) { return v.size(); }
inline Vector ediv(const Vector &a, const Vector &b) {assert (b.size()==a.size()); return a.cwiseQuotient(b);}
inline Vector esqrt(const Vector& v) { return v.cwiseSqrt();}
inline Vector emul(const Vector &a, const Vector &b) {assert (b.size()==a.size()); return a.cwiseProduct(b);}
inline double max(const Vector &a){return a.maxCoeff();}
inline double norm_2(const Vector& v) {return v.norm();}
inline Vector ones(size_t n) { return Vector::Ones(n); }
inline Vector reciprocal(const Vector &a) {return a.array().inverse();}
inline Vector repeat(size_t n, double value) {return Vector::Constant(n, value);}
inline const Vector sub(const Vector &v, size_t i1, size_t i2) {return v.segment(i1,i2-i1);}
inline void subInsert(Vector& fullVector, const Vector& subVector, size_t i) {fullVector.segment(i, subVector.size()) = subVector;}
inline double sum(const Vector &a){return a.sum();}
inline bool zero(const Vector& v){ return v.isZero(); }
inline Vector zero(size_t n) { return Vector::Zero(n); }
#endif
} // namespace gtsam

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
  namespace serialization {

    // split version - copies into an STL vector for serialization
    template<class Archive>
    void save(Archive & ar, const gtsam::Vector & v, unsigned int /*version*/) {
      const size_t size = v.size();
      ar << BOOST_SERIALIZATION_NVP(size);
      ar << make_nvp("data", make_array(v.data(), v.size()));
    }

    template<class Archive>
    void load(Archive & ar, gtsam::Vector & v, unsigned int /*version*/) {
      size_t size;
      ar >> BOOST_SERIALIZATION_NVP(size);
      v.resize(size);
      ar >> make_nvp("data", make_array(v.data(), v.size()));
    }

    // split version - copies into an STL vector for serialization
    template<class Archive, int D>
    void save(Archive & ar, const Eigen::Matrix<double,D,1> & v, unsigned int /*version*/) {
      ar << make_nvp("data", make_array(v.data(), v.RowsAtCompileTime));
    }

    template<class Archive, int D>
    void load(Archive & ar, Eigen::Matrix<double,D,1> & v, unsigned int /*version*/) {
      ar >> make_nvp("data", make_array(v.data(), v.RowsAtCompileTime));
    }

  } // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector2)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector3)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector6)
