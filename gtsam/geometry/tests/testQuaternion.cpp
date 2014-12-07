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

namespace group {

template<typename S, int O>
Eigen::Quaternion<S, O> compose(const Eigen::Quaternion<S, O> &g,
    const Eigen::Quaternion<S, O> & h) {
  return g * h;
}

template<typename S, int O>
Eigen::Quaternion<S, O> between(const Eigen::Quaternion<S, O> &g,
    const Eigen::Quaternion<S, O> & h) {
  return g.inverse() * h;
}

template<typename S, int O>
Eigen::Quaternion<S, O> inverse(const Eigen::Quaternion<S, O> &g) {
  return g.inverse();
}

namespace traits {

/// Declare the trait that specifies a quaternion's identity element
template<typename S, int O>
struct identity<Eigen::Quaternion<S, O> > {
  static const Eigen::Quaternion<S, O> value;
  typedef Eigen::Quaternion<S, O> value_type;
};

/// Out of line definition of identity
template<typename S, int O>
const Eigen::Quaternion<S, O> identity<Eigen::Quaternion<S, O> >::value =
    Eigen::Quaternion<S, O>::Identity();

/// Define the trait that asserts quaternions are a multiplicative group
template<typename S, int O>
struct flavor<Eigen::Quaternion<S, O> > {
  typedef multiplicative_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

/**
 *  GSAM typedef to an Eigen::Quaternion<double>, we disable alignment because
 *  geometry objects are stored in boost pool allocators, in Values containers,
 *  and and these pool allocators do not support alignment.
 */
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

/**
 * @file   testCyclic.cpp
 * @brief  Unit tests for cyclic group
 * @author Frank Dellaert
 **/

//#include <gtsam/geometry/Quaternion.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Quaternion Q; // Typedef

//******************************************************************************
TEST(Quaternion , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<Quaternion >));
  BOOST_CONCEPT_ASSERT((IsManifold<Quaternion >));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Quaternion >));
}

//******************************************************************************
TEST(Quaternion , Constructor) {
  Q q(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
TEST(Quaternion , Invariants) {
  Q q1(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
  Q q2(Eigen::AngleAxisd(2, Vector3(0, 1, 0)));
  // group::check_invariants(q1,q2); Does not satisfy Testable concept (yet!)
}

//******************************************************************************
TEST(Quaternion , Local) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0, z_axis));
  Q q2(Eigen::AngleAxisd(0.1, z_axis));
  typedef manifold::traits::DefaultChart<Q>::type Chart;
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = Chart::Local(q1, q2);
  EXPECT(assert_equal((Vector)expected,actual));
}

//******************************************************************************
TEST(Quaternion , Retract) {
  Vector3 z_axis(0, 0, 1);
  Q q(Eigen::AngleAxisd(0, z_axis));
  Q expected(Eigen::AngleAxisd(0.1, z_axis));
  typedef manifold::traits::DefaultChart<Q>::type Chart;
  Vector3 v(0, 0, 0.1);
  Q actual = Chart::Retract(q, v);
  EXPECT(actual.isApprox(expected));
}

//******************************************************************************
TEST(Quaternion , Compose) {
}

//******************************************************************************
TEST(Quaternion , Between) {
}

//******************************************************************************
TEST(Quaternion , Inverse) {
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

