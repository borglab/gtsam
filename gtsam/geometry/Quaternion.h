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
#include <gtsam/base/Matrix.h>

#define QUATERNION_TEMPLATE typename _Scalar, int _Options
#define QUATERNION_TYPE Eigen::Quaternion<_Scalar,_Options>

namespace gtsam {

// Define group traits
template<QUATERNION_TEMPLATE>
struct traits_x<QUATERNION_TYPE> {
  typedef QUATERNION_TYPE ManifoldType;
  typedef QUATERNION_TYPE Q;
  typedef lie_group_tag structure_category;
  typedef multiplicative_group_tag group_flavor;

  enum { dimension = 3 };

  static Q Identity() { return Q::Identity(); }

// Define manifold traits
  typedef Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1> TangentVector;
  typedef OptionalJacobian<3, 3> ChartJacobian;

  static Q Compose(const Q &g, const Q & h, ChartJacobian Hg=boost::none, ChartJacobian Hh=boost::none) {
    if (Hg) {
      //TODO : check Jacobian consistent with chart ( h.toRotationMatrix().transpose() ? )
      *Hg = h.toRotationMatrix().transpose();
    }
    if (Hh) {
      //TODO : check Jacobian consistent with chart ( I(3)? )
      *Hh = I_3x3;
    }
    return g * h;
  }
  static Q Between(const Q &g, const Q & h, ChartJacobian Hg=boost::none, ChartJacobian Hh=boost::none) {
    Q d = g.inverse() * h;
    if (Hg) {
        //TODO : check Jacobian consistent with chart
      *Hg = -d.toRotationMatrix().transpose();
      }
      if (Hh) {
        //TODO : check Jacobian consistent with chart (my guess I(3) )
        *Hh = I_3x3;
      }
      return d;
  }
  static Q Inverse(const Q &g, ChartJacobian H=boost::none) {
      if (H) {
        //TODO : check Jacobian consistent with chart
        *H = -g.toRotationMatrix();
       }
       return g.inverse();
  }

  /// Exponential map, simply be converting omega to axis/angle representation
  // TODO: implement Jacobian
  static Q Expmap(const Eigen::Ref<const TangentVector >& omega, ChartJacobian H=boost::none) {
    if (omega.isZero())
      return Q::Identity();
    else {
      _Scalar angle = omega.norm();
      return Q(Eigen::AngleAxis<_Scalar>(angle, omega / angle));
    }
  }

  /// We use our own Logmap, as there is a slight bug in Eigen
  // TODO: implement Jacobian
  static TangentVector Logmap(const Q& q, ChartJacobian H=boost::none) {
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

  static TangentVector Local(const Q& origin, const Q& other, ChartJacobian Horigin=boost::none, ChartJacobian Hother=boost::none) {
    return Logmap(Between(origin,other,Horigin,Hother));
    // TODO: incorporate Jacobian of Logmap
  }
  static Q Retract(const Q& origin, const TangentVector& v, ChartJacobian Horigin, ChartJacobian Hv) {
    return Compose(origin, Expmap(v), Horigin, Hv);
    // TODO : incorporate Jacobian of Expmap
  }
};

typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

} // \namespace gtsam

