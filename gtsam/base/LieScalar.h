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

#ifdef _MSC_VER
#pragma message("LieScalar.h is deprecated. Please use double/float instead.")
#else
  #warning "LieScalar.h is deprecated. Please use double/float instead."
#endif

#include <gtsam/dllexport.h>
#include <gtsam/base/VectorSpace.h>

namespace gtsam {

  /**
   * @deprecated: LieScalar, LieVector and LieMatrix are obsolete in GTSAM 4.0 as
   * we can directly add double, Vector, and Matrix into values now, because of
   * gtsam::traits.
   */
  struct GTSAM_EXPORT LieScalar {

    enum { dimension = 1 };

    /** default constructor */
    LieScalar() : d_(0.0) {}

    /** wrap a double */
    /*explicit*/ LieScalar(double d) : d_(d) {}

    /** access the underlying value */
    double value() const { return d_; }

    /** Automatic conversion to underlying value */
    operator double() const { return d_; }

    /// @name Testable
    /// @{

    /** print @param name optional string naming the object */
    void print(const std::string& name="") const;

    /** equality up to tolerance */
    bool equals(const LieScalar& expected, double tol=1e-5) const {
      return fabs(expected.d_ - d_) <= tol;
    }

    /// @}

  private:
      double d_;
  };

  template<>
  struct traits_x<LieScalar> : public internal::ScalarTraits<LieScalar> {};

} // \namespace gtsam
