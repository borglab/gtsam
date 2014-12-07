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

namespace traits {

/// Define Eigen::Quaternion to be a model of the Lie Group concept
template<typename S, int O>
struct structure_category<Eigen::Quaternion<S, O> > {
  typedef lie_group_tag type;
};

} // \namespace gtsam::traits

namespace manifold {

/// Chart for Eigen Quaternions
template<typename S, int O>
struct QuaternionChart {

  // required
  typedef Eigen::Quaternion<S, O> ManifoldType;

  // internal
  typedef ManifoldType Q;
  typedef typename traits::TangentVector<Q>::type Omega;

  /// Exponential map, simply be converting omega to AngleAxis
  static Q Expmap(const Omega& omega) {
    double theta = omega.norm();
    if (std::abs(theta) < 1e-10)
      return Q::Identity();
    return Q(Eigen::AngleAxisd(theta, omega / theta));
  }

  /// retract, simply be converting omega to AngleAxis
  static Q Retract(const Q& p, const Omega& omega) {
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
      return Vector3::Zero();
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

namespace traits {

/// Define the trait that asserts Quaternion manifold has dimension 3
template<typename S, int O>
struct dimension<Eigen::Quaternion<S, O> > : public boost::integral_constant<
    int, 3> {
};

/// Define the trait that asserts Quaternion TangentVector is Vector3
template<typename S, int O>
struct TangentVector<Eigen::Quaternion<S, O> > {
  typedef Eigen::Matrix<S, 3, 1, O, 3, 1> type;
};

/// Define the trait that asserts Quaternion TangentVector is Vector3
template<typename S, int O>
struct DefaultChart<Eigen::Quaternion<S, O> > {
  typedef QuaternionChart<S, O> type;
};

} // \namespace gtsam::manifold::traits
} // \namespace gtsam::manifold

GTSAM_MULTIPLICATIVE_GROUP2(typename,_Scalar, int,_Options ,Eigen::Quaternion)

} // \namespace gtsam

/**
 *  GSAM typedef to an Eigen::Quaternion<double>, we disable alignment because
 *  geometry objects are stored in boost pool allocators, in Values containers,
 *  and and these pool allocators do not support alignment.
 */
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

