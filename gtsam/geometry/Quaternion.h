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

#define QUATERNION_TYPE Eigen::Quaternion<_Scalar,_Options>

namespace gtsam {

// Define traits
template<typename _Scalar, int _Options>
struct traits_x<QUATERNION_TYPE> {
  typedef QUATERNION_TYPE ManifoldType;
  typedef QUATERNION_TYPE Q;

  typedef lie_group_tag structure_category;
  typedef multiplicative_group_tag group_flavor;

  /// @name Basic Manifold traits
  /// @{
  enum {
    dimension = 3
  };
  typedef OptionalJacobian<3, 3> ChartJacobian;
  typedef Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1> TangentVector;

  /// @}
  /// @name Lie group traits
  /// @{
  static Q Identity() {
    return Q::Identity();
  }

  static Q Compose(const Q &g, const Q & h, ChartJacobian Hg = boost::none,
      ChartJacobian Hh = boost::none) {
    if (Hg)
      *Hg = h.toRotationMatrix().transpose(); // TODO : check Jacobian consistent with chart ( h.toRotationMatrix().transpose() ? )
    if (Hh)
      *Hh = I_3x3; // TODO : check Jacobian consistent with chart ( I(3)? )
    return g * h;
  }

  static Q Between(const Q &g, const Q & h, ChartJacobian Hg = boost::none,
      ChartJacobian Hh = boost::none) {
    Q d = g.inverse() * h;
    if (Hg)
      *Hg = -d.toRotationMatrix().transpose(); // TODO : check Jacobian consistent with chart
    if (Hh)
      *Hh = I_3x3; // TODO : check Jacobian consistent with chart (my guess I(3) )
    return d;
  }

  static Q Inverse(const Q &g, ChartJacobian H = boost::none) {
    if (H)
      *H = -g.toRotationMatrix(); // TODO : check Jacobian consistent with chart
    return g.inverse();
  }

  /// Exponential map, simply be converting omega to axis/angle representation
  static Q Expmap(const Eigen::Ref<const TangentVector>& omega,
      ChartJacobian H = boost::none) {
    if (omega.isZero())
      return Q::Identity();
    else {
      _Scalar angle = omega.norm();
      return Q(Eigen::AngleAxis<_Scalar>(angle, omega / angle));
    }
    if (H)
      throw std::runtime_error("TODO: implement Jacobian");
  }

  /// We use our own Logmap, as there is a slight bug in Eigen
  static TangentVector Logmap(const Q& q, ChartJacobian H = boost::none) {
    using std::acos;
    using std::sqrt;

    // define these compile time constants to avoid std::abs:
    static const double twoPi = 2.0 * M_PI, NearlyOne = 1.0 - 1e-10,
        NearlyNegativeOne = -1.0 + 1e-10;

    const double qw = q.w();
    if (qw > NearlyOne) {
      // Taylor expansion of (angle / s) at 1
      //return (2 + 2 * (1-qw) / 3) * q.vec();
      return (8. / 3. - 2. / 3. * qw) * q.vec();
    } else if (qw < NearlyNegativeOne) {
      // Taylor expansion of (angle / s) at -1
      //return (-2 - 2 * (1 + qw) / 3) * q.vec();
      return (-8. / 3 + 2. / 3 * qw) * q.vec();
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

    if (H)
      throw std::runtime_error("TODO: implement Jacobian");
  }

  /// @}
  /// @name Manifold traits
  /// @{
  static TangentVector Local(const Q& origin, const Q& other,
      ChartJacobian Horigin = boost::none, ChartJacobian Hother = boost::none) {
    return Logmap(Between(origin, other, Horigin, Hother));
    // TODO: incorporate Jacobian of Logmap
  }
  static Q Retract(const Q& origin, const TangentVector& v,
      ChartJacobian Horigin = boost::none, ChartJacobian Hv = boost::none) {
    return Compose(origin, Expmap(v), Horigin, Hv);
    // TODO : incorporate Jacobian of Expmap
  }

  /// @}
  /// @name Testable
  /// @{
  static void Print(const Q& q, const std::string& str = "") {
    if (str.size() == 0)
      std::cout << "Eigen::Quaternion: ";
    else
      std::cout << str << " ";
    std::cout << q.vec().transpose() << std::endl;
  }
  static bool Equals(const Q& q1, const Q& q2, double tol = 1e-8) {
    return Between(q1, q2).vec().array().abs().maxCoeff() < tol;
  }
  /// @}
};

typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

} // \namespace gtsam

