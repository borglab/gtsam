/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieScalar.h
 * @brief A wrapper around scalar providing Lie compatibility
 * @author Kai Ni
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/VectorSpace.h>
#include <iostream>

namespace gtsam {

  /**
   * @deprecated: LieScalar, LieVector and LieMatrix are obsolete in GTSAM 4.0 as
   * we can directly add double, Vector, and Matrix into values now, because of
   * gtsam::traits.
   */
  struct LieScalar {

    enum { dimension = 1 };

    /** default constructor */
    LieScalar() : d_(0.0) {}

    /** wrap a double */
    /*explicit*/ LieScalar(double d) : d_(d) {}

    /** access the underlying value */
    double value() const { return d_; }

    /** Automatic conversion to underlying value */
    operator double() const { return d_; }

    /** convert vector */
    Vector1 vector() const { Vector1 v; v<<d_; return v; }

    /// @name Testable
    /// @{
    void print(const std::string& name = "") const {
      std::cout << name << ": " << d_ << std::endl;
    }
    bool equals(const LieScalar& expected, double tol = 1e-5) const {
      return std::abs(expected.d_ - d_) <= tol;
    }
    /// @}

    /// @name Group
    /// @{
    static LieScalar identity() { return LieScalar(0);}
    LieScalar compose(const LieScalar& q) { return (*this)+q;}
    LieScalar between(const LieScalar& q) { return q-(*this);}
    LieScalar inverse() { return -(*this);}
    /// @}

    /// @name Manifold
    /// @{
    size_t dim() const { return 1; }
    Vector1 localCoordinates(const LieScalar& q) { return between(q).vector();}
    LieScalar retract(const Vector1& v) {return compose(LieScalar(v[0]));}
    /// @}

    /// @name Lie Group
    /// @{
    static Vector1 Logmap(const LieScalar& p) { return p.vector();}
    static LieScalar Expmap(const Vector1& v) { return LieScalar(v[0]);}
    /// @}

  private:
      double d_;
  };

  template<>
  struct traits<LieScalar> : public internal::ScalarTraits<LieScalar> {};

} // \namespace gtsam
