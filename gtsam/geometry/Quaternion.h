/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Quaternion.h
 * @brief  Unit tests for unit quaternions
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>

namespace gtsam {

/// Chart for Eigen Quaternions
template<typename _Scalar, int _Options>
struct MakeQuaternionChart {

  // required
  typedef Eigen::Quaternion<_Scalar, _Options> ManifoldType;

  // internal
  typedef ManifoldType Q;
  typedef typename manifold::traits::TangentVector<Q>::type Omega;

  /// Exponential map given axis/angle representation of Lie algebra
  static Q Expmap(const _Scalar& angle, const Eigen::Ref<const Omega>& axis) {
    return Q(Eigen::AngleAxisd(angle, axis));
  }

  /// Exponential map, simply be converting omega to axis/angle representation
  static Q Expmap(const Eigen::Ref<const Omega>& omega) {
    if (omega.isZero())
      return Q::Identity();
    else {
      _Scalar angle = omega.norm();
      return Q(Eigen::AngleAxisd(angle, omega / angle));
    }
  }

  /// retract, simply be converting omega to AngleAxis
  static Q Retract(const Q& p, const Eigen::Ref<const Omega>& omega) {
    return p * Expmap(omega);
  }

  /// We use our own Logmap, as there is a slight bug in Eigen
  static Omega Logmap(const Q& q) {
    using std::acos;
    using std::sqrt;
    static const double twoPi = 2.0 * M_PI,
    // define these compile time constants to avoid std::abs:
        NearlyOne = 1.0 - 1e-10, NearlyNegativeOne = -1.0 + 1e-10;

    const double qw = q.w();
    if (qw > NearlyOne) {
      // Taylor expansion of (angle / s) at 1
      return (2 - 2 * (qw - 1) / 3) * q.vec();
    } else if (qw < NearlyNegativeOne) {
      // Angle is zero, return zero vector
      return Omega::Zero();
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

  /// local is our own, as there is a slight bug in Eigen
  static Omega Local(const Q& q1, const Q& q2) {
    return Logmap(q1.inverse() * q2);
  }
};

#define QUATERNION_TEMPLATE typename _Scalar, int _Options
#define QUATERNION_TYPE Eigen::Quaternion<_Scalar,_Options>
GTSAM_MULTIPLICATIVE_GROUP(QUATERNION_TEMPLATE, QUATERNION_TYPE)

#define QUATERNION_TANGENT Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1>
#define QUATERNION_CHART MakeQuaternionChart<_Scalar,_Options>
GTSAM_MANIFOLD(QUATERNION_TEMPLATE, QUATERNION_TYPE, 3, QUATERNION_TANGENT,
    QUATERNION_CHART)

/// Define Eigen::Quaternion to be a model of the Lie Group concept
namespace traits {
template<typename S, int O>
struct structure_category<Eigen::Quaternion<S, O> > {
  typedef lie_group_tag type;
};
}

/**
 *  GSAM typedef to an Eigen::Quaternion<double>, we disable alignment because
 *  geometry objects are stored in boost pool allocators, in Values containers,
 *  and and these pool allocators do not support alignment.
 */
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;
typedef MakeQuaternionChart<double, Eigen::DontAlign> QuaternionChart;

} // \namespace gtsam

