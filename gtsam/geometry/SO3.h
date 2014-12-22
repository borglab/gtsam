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
class SO3: public Matrix3 {

protected:

public:
  enum { dimension=3 };

  /// Constructor from Eigen Matrix
  template<typename Derived>
  SO3(const MatrixBase<Derived>& R) :
      Matrix3(R.eval()) {
  }

  /// Constructor from AngleAxisd
  SO3(const Eigen::AngleAxisd& angleAxis) :
      Matrix3(angleAxis) {
  }

  static SO3 identity() {
    return I_3x3;
  }

};

template<>
struct traits_x<SO3> : public internal::LieGroup<SO3> {};


} // end namespace gtsam

