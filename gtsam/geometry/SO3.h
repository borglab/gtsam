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
#include <gtsam/base/concepts.h>

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

  /// Constructor from Eigen Matrix
  template<typename Derived>
  SO3(const MatrixBase<Derived>& R) :
      Matrix3(R.eval()) {
  }

  /// Constructor from AngleAxisd
  SO3(const Eigen::AngleAxisd& angleAxis) :
      Matrix3(angleAxis) {
  }
};

#define SO3_TEMPLATE
GTSAM_GROUP_IDENTITY0(SO3)
GTSAM_MULTIPLICATIVE_GROUP(SO3_TEMPLATE, SO3)

/**
 * Chart for SO3 comprising of exponential map and its inverse (log-map)
 */
typedef LieGroupChart<SO3> SO3Chart;

#define SO3_TANGENT Vector3
#define SO3_CHART SO3Chart
GTSAM_MANIFOLD(SO3_TEMPLATE, SO3, 3, SO3_TANGENT, SO3_CHART)

/// Define SO3 to be a model of the Lie Group concept
namespace traits {
template<>
struct structure_category<SO3> {
  typedef lie_group_tag type;
};
}

} // end namespace gtsam

