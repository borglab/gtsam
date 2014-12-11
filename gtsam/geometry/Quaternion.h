/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Quaternion.h
 * @brief  Lie Group wrapper for Eigen Quaternions
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>

#define QUATERNION_TEMPLATE typename _Scalar, int _Options
#define QUATERNION_TYPE Eigen::Quaternion<_Scalar,_Options>

template<QUATERNION_TEMPLATE>
QUATERNION_TYPE operator-(const QUATERNION_TYPE &q) { return gtsam::group::inverse(q); }

namespace gtsam {

// Define group traits
GTSAM_GROUP_IDENTITY(QUATERNION_TEMPLATE, QUATERNION_TYPE)
GTSAM_MULTIPLICATIVE_GROUP(QUATERNION_TEMPLATE, QUATERNION_TYPE)

// Define manifold traits
#define QUATERNION_TANGENT Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1>
#define QUATERNION_CHART LieGroupChart<typename QUATERNION_TYPE>
GTSAM_MANIFOLD(QUATERNION_TEMPLATE, QUATERNION_TYPE, 3, QUATERNION_TANGENT,
    QUATERNION_CHART)

/// Define Eigen::Quaternion to be a model of the Lie Group concept
namespace traits {
template<typename _Scalar, int _Options>
struct structure_category<Eigen::Quaternion<_Scalar,_Options> > {
  typedef lie_group_tag type;
};
}

/// lie_group for Eigen Quaternions
namespace lie_group {

  /// Exponential map, simply be converting omega to axis/angle representation
  template <typename _Scalar, int _Options>
  static QUATERNION_TYPE expmap<QUATERNION_TYPE>(const Eigen::Ref<const typename QUATERNION_TANGENT >& omega) {
    if (omega.isZero())
      return QUATERNION_TYPE::Identity();
    else {
      _Scalar angle = omega.norm();
      return QUATERNION_TYPE(Eigen::AngleAxis<_Scalar>(angle, omega / angle));
    }
  }

  /// We use our own Logmap, as there is a slight bug in Eigen
  template <QUATERNION_TEMPLATE>
  static QUATERNION_TANGENT logmap<QUATERNION_TYPE>(const QUATERNION_TYPE& q) {
    using std::acos;
    using std::sqrt;
    static const double twoPi = 2.0 * M_PI,
    // define these compile time constants to avoid std::abs:
        NearlyOne = 1.0 - 1e-10, NearlyNegativeOne = -1.0 + 1e-10;

    const double qw = q.w();
    if (qw > NearlyOne) {
      // Taylor expansion of (angle / s) at 1
      //return (2 + 2 * (1-qw) / 3) * q.vec();
      return (8./3. - 2./3. * qw) * q.vec();
    } else if (qw < NearlyNegativeOne) {
      // Taylor expansion of (angle / s) at -1
      //return (-2 - 2 * (1 + qw) / 3) * q.vec();
      return (-8./3 + 2./3 * qw) * q.vec();
    } else {
      // Normal, away from zero case
      double angle = 2 * acos(qw), s = sqrt(1 - qw * qw);
      // Important:  convert to [-pi,pi] to keep error continuous
      if (angle > M_PI)
        angle -= twoPi;
      else if (angle < -M_PI)
        angle += twoPi;
      return (angle / s) * q.vec();
    }
  }
} // end namespace lie_group

/**
 *  GSAM typedef to an Eigen::Quaternion<double>, we disable alignment because
 *  geometry objects are stored in boost pool allocators, in Values containers,
 *  and and these pool allocators do not support alignment.
 */
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;
typedef LieGroupChart<Quaternion> QuaternionChart;

} // \namespace gtsam

