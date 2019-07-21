/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieVector.h
 * @brief A wrapper around vector providing Lie compatibility
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/VectorSpace.h>
#include <cstdarg>

namespace gtsam {

/**
 * @deprecated: LieVector, LieVector and LieMatrix are obsolete in GTSAM 4.0 as
 * we can directly add double, Vector, and Matrix into values now, because of
 * gtsam::traits.
 */
struct LieVector : public Vector {

  enum { dimension = Eigen::Dynamic };

  /** default constructor - should be unnecessary */
  LieVector() {}

  /** initialize from a normal vector */
  LieVector(const Vector& v) : Vector(v) {}

  template <class V>
  LieVector(const V& v) : Vector(v) {}

// Currently TMP constructor causes ICE on MSVS 2013
#if (_MSC_VER < 1800)
  /** initialize from a fixed size normal vector */
  template<int N>
  LieVector(const Eigen::Matrix<double, N, 1>& v) : Vector(v) {}
#endif

  /** wrap a double */
  LieVector(double d) : Vector((Vector(1) << d).finished()) {}

  /** constructor with size and initial data, row order ! */
  LieVector(size_t m, const double* const data) : Vector(m) {
    for (size_t i = 0; i < m; i++) (*this)(i) = data[i];
  }

  /// @name Testable
  /// @{
  void print(const std::string& name="") const {
    gtsam::print(vector(), name);
  }
  bool equals(const LieVector& expected, double tol=1e-5) const {
    return gtsam::equal(vector(), expected.vector(), tol);
  }
  /// @}

  /// @name Group
  /// @{
  LieVector compose(const LieVector& q) { return (*this)+q;}
  LieVector between(const LieVector& q) { return q-(*this);}
  LieVector inverse() { return -(*this);}
  /// @}

  /// @name Manifold
  /// @{
  Vector localCoordinates(const LieVector& q) { return between(q).vector();}
  LieVector retract(const Vector& v) {return compose(LieVector(v));}
  /// @}

  /// @name Lie Group
  /// @{
  static Vector Logmap(const LieVector& p) {return p.vector();}
  static LieVector Expmap(const Vector& v) { return LieVector(v);}
  /// @}

  /// @name VectorSpace requirements
  /// @{

  /** get the underlying vector */
  Vector vector() const {
    return static_cast<Vector>(*this);
  }

  /** Returns dimensionality of the tangent space */
  size_t dim() const { return this->size(); }

  /** identity - NOTE: no known size at compile time - so zero length */
  static LieVector identity() {
    throw std::runtime_error("LieVector::identity(): Don't use this function");
    return LieVector();
  }

  /// @}

private:

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("Vector",
       boost::serialization::base_object<Vector>(*this));
  }
};


template<>
struct traits<LieVector> : public internal::VectorSpace<LieVector> {};

} // \namespace gtsam
