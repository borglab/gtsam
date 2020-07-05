/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieMatrix.h
 * @brief A wrapper around Matrix providing Lie compatibility
 * @author Richard Roberts and Alex Cunningham
 */

#pragma once

#include <cstdarg>

#include <gtsam/base/VectorSpace.h>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

/**
 * @deprecated: LieMatrix, LieVector and LieMatrix are obsolete in GTSAM 4.0 as
 * we can directly add double, Vector, and Matrix into values now, because of
 * gtsam::traits.
 */
struct LieMatrix : public Matrix {

  /// @name Constructors
  /// @{
  enum { dimension = Eigen::Dynamic };

  /** default constructor - only for serialize */
  LieMatrix() {}

  /** initialize from a normal matrix */
  LieMatrix(const Matrix& v) : Matrix(v) {}

  template <class M>
  LieMatrix(const M& v) : Matrix(v) {}

// Currently TMP constructor causes ICE on MSVS 2013
#if (_MSC_VER < 1800)
  /** initialize from a fixed size normal vector */
  template<int M, int N>
  LieMatrix(const Eigen::Matrix<double, M, N>& v) : Matrix(v) {}
#endif

  /** constructor with size and initial data, row order ! */
  LieMatrix(size_t m, size_t n, const double* const data) :
      Matrix(Eigen::Map<const Matrix>(data, m, n)) {}

  /// @}
  /// @name Testable interface
  /// @{

  /** print @param s optional string naming the object */
  void print(const std::string& name = "") const {
    gtsam::print(matrix(), name);
  }
  /** equality up to tolerance */
  inline bool equals(const LieMatrix& expected, double tol=1e-5) const {
    return gtsam::equal_with_abs_tol(matrix(), expected.matrix(), tol);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /** get the underlying matrix */
  inline Matrix matrix() const {
    return static_cast<Matrix>(*this);
  }

  /// @}

  /// @name Group
  /// @{
  LieMatrix compose(const LieMatrix& q) { return (*this)+q;}
  LieMatrix between(const LieMatrix& q) { return q-(*this);}
  LieMatrix inverse() { return -(*this);}
  /// @}

  /// @name Manifold
  /// @{
  Vector localCoordinates(const LieMatrix& q) { return between(q).vector();}
  LieMatrix retract(const Vector& v) {return compose(LieMatrix(v));}
  /// @}

  /// @name Lie Group
  /// @{
  static Vector Logmap(const LieMatrix& p) {return p.vector();}
  static LieMatrix Expmap(const Vector& v) { return LieMatrix(v);}
  /// @}

  /// @name VectorSpace requirements
  /// @{

  /** Returns dimensionality of the tangent space */
  inline size_t dim() const { return size(); }

  /** Convert to vector, is done row-wise - TODO why? */
  inline Vector vector() const {
    Vector result(size());
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor> RowMajor;
    Eigen::Map<RowMajor>(&result(0), rows(), cols()) = *this;
    return result;
  }

  /** identity - NOTE: no known size at compile time - so zero length */
  inline static LieMatrix identity() {
    throw std::runtime_error("LieMatrix::identity(): Don't use this function");
    return LieMatrix();
  }
  /// @}

private:

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("Matrix",
       boost::serialization::base_object<Matrix>(*this));

  }

};


template<>
struct traits<LieMatrix> : public internal::VectorSpace<LieMatrix> {

  // Override Retract, as the default version does not know how to initialize
  static LieMatrix Retract(const LieMatrix& origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(origin);
    if (H2) *H2 = Eye(origin);
    typedef const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor> RowMajor;
    return origin + Eigen::Map<RowMajor>(&v(0), origin.rows(), origin.cols());
  }

};

} // \namespace gtsam
