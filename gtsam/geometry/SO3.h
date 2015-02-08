/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.h
 * @brief   3*3 matrix representation of SO(3)
 * @author  Frank Dellaert
 * @date    December 2014
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

#include <cmath>

namespace gtsam {

/**
 *  True SO(3), i.e., 3*3 matrix subgroup
 *  We guarantee (all but first) constructors only generate from sub-manifold.
 *  However, round-off errors in repeated composition could move off it...
 */
class GTSAM_EXPORT SO3: public Matrix3, public LieGroup<SO3,3> {

protected:

public:
  enum { dimension=3 };

  /// Constructor from AngleAxisd
  SO3() : Matrix3(I_3x3) {
  }

  /// Constructor from Eigen Matrix
  template<typename Derived>
  SO3(const MatrixBase<Derived>& R) :
      Matrix3(R.eval()) {
  }

  /// Constructor from AngleAxisd
  SO3(const Eigen::AngleAxisd& angleAxis) :
      Matrix3(angleAxis) {
  }

  void print(const std::string& s) const {
    std::cout << s << *this << std::endl;
  }

  bool equals(const SO3 & R, double tol) const {
    return equal_with_abs_tol(*this, R, tol);
  }

  static SO3 identity() { return I_3x3; }
  SO3 inverse() const { return this->Matrix3::inverse(); }

  static SO3 Expmap(const Eigen::Ref<const Vector3>& omega, ChartJacobian H = boost::none);
  static Vector3 Logmap(const SO3& R, ChartJacobian H = boost::none);

  Matrix3 AdjointMap() const { return *this; }

  // Chart at origin
  struct ChartAtOrigin {
    static SO3 Retract(const Vector3& v, ChartJacobian H = boost::none) {
      return Expmap(v,H);
    }
    static Vector3 Local(const SO3& R, ChartJacobian H = boost::none) {
      return Logmap(R,H);
    }
  };

  using LieGroup<SO3,3>::inverse;

};

template<>
struct traits<SO3> : public internal::LieGroupTraits<SO3> {};

template<>
struct traits<const SO3> : public internal::LieGroupTraits<SO3> {};
} // end namespace gtsam

