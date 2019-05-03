/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSOnBase.cpp
 * @brief  Unit tests for Base class of SO(n) classes.
 * @author Frank Dellaert
 **/

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>

#include <iostream>
#include <stdexcept>
#include <type_traits>

namespace gtsam {

namespace internal {
// Calculate dimensionality of SO<N> manifold, or return Dynamic if so
constexpr int DimensionSO(int N) {
  return (N < 0) ? Eigen::Dynamic : N * (N - 1) / 2;
}
}  // namespace internal

template <int N>
class SO : public Eigen::Matrix<double, N, N>,
           public LieGroup<SO<N>, internal::DimensionSO(N)> {
 public:
  enum { dimension = internal::DimensionSO(N) };
  using MatrixNN = Eigen::Matrix<double, N, N>;
  using VectorD = Eigen::Matrix<double, dimension, 1>;
  using MatrixDD = Eigen::Matrix<double, dimension, dimension>;

  /// @name Constructors
  /// @{

  /// Construct SO<N> identity for N >= 2
  template <int N_ = N, typename = boost::enable_if_t<N_ >= 2, void>>
  SO() {
    *this << MatrixNN::Identity();
  }

  /// Construct SO<N> identity for N == Eigen::Dynamic
  template <int N_ = N,
            typename = boost::enable_if_t<N_ == Eigen::Dynamic, void>>
  explicit SO(size_t n = 0) {
    if (n == 0) throw std::runtime_error("SO: Dimensionality not known.");
    this->resize(n, n);
    *this << Eigen::MatrixXd::Identity(n, n);
  }

  /// Constructor from Eigen Matrix
  template <typename Derived>
  SO(const Eigen::MatrixBase<Derived>& R) : MatrixNN(R.eval()) {}

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s) const {
    std::cout << s << *this << std::endl;
  }

  bool equals(const SO& R, double tol) const {
    return equal_with_abs_tol(*this, R, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /// SO<N> identity for N >= 2
  template <int N_ = N, typename = boost::enable_if_t<N_ >= 2, void>>
  static SO identity() {
    return SO();
  }

  /// SO<N> identity for N == Eigen::Dynamic
  template <int N_ = N,
            typename = boost::enable_if_t<N_ == Eigen::Dynamic, void>>
  static SO identity(size_t n = 0) {
    return SO(n);
  }

  /// inverse of a rotation = transpose
  SO inverse() const { return this->transpose(); }

  /// @}
  /// @name Manifold
  /// @{

  // Chart at origin
  struct ChartAtOrigin {
    static SO Retract(const VectorD& xi,
                      OptionalJacobian<dimension, dimension> H = boost::none) {
      return SO();  // TODO(frank): Expmap(xi, H);
    }
    static VectorD Local(
        const SO& R, OptionalJacobian<dimension, dimension> H = boost::none) {
      return VectorD();  // TODO(frank): Logmap(R, H);
    }
  };

  /// @}
  /// @name Lie Group
  /// @{

  MatrixDD AdjointMap() const { return MatrixDD(); }

  /**
   * Exponential map at identity - create a rotation from canonical coordinates
   */
  static SO Expmap(const VectorD& omega, OptionalJacobian<dimension, dimension> H = boost::none);

  /**
   * Log map at identity - returns the canonical coordinates of this rotation
   */
  static VectorD Logmap(const SO& R, OptionalJacobian<dimension, dimension> H = boost::none);

  // inverse with optional derivative
  using LieGroup<SO<N>, internal::DimensionSO(N)>::inverse;

  /// @}
};

using SOn = SO<Eigen::Dynamic>;
using SO3 = SO<3>;
using SO4 = SO<4>;

template <int N>
struct traits<SO<N>> : public internal::LieGroup<SO<N>> {};

template <int N>
struct traits<const SO<N>> : public internal::LieGroup<SO<N>> {};

}  // namespace gtsam

// #include <gtsam/base/Manifold.h>
// #include <gtsam/base/Testable.h>
// #include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
// #include <gtsam/geometry/SO3.h>
// #include <gtsam/geometry/SO4.h>
// #include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// TEST(SOn, SO5) {
//   const auto R = SOn(5);
//   EXPECT_LONGS_EQUAL(5, R.rows());
// }

/* ************************************************************************* */
TEST(SOn, Concept) {
  // BOOST_CONCEPT_ASSERT((IsGroup<SOn>));
  // BOOST_CONCEPT_ASSERT((IsManifold<SOn>));
  // BOOST_CONCEPT_ASSERT((IsLieGroup<SOn>));
}

// /* *************************************************************************
// */ TEST(SOn, Values) {
//   const auto R = SOn(5);
//   Values values;
//   const Key key(0);
//   values.insert(key, R);
//   const auto B = values.at<SOn>(key);
//   EXPECT_LONGS_EQUAL(5, B.rows());
// }

// /* *************************************************************************
// */ TEST(SOn, Random) {
//   static boost::mt19937 rng(42);
//   EXPECT_LONGS_EQUAL(3, SOn::Random(rng, 3).rows());
//   EXPECT_LONGS_EQUAL(4, SOn::Random(rng, 4).rows());
//   EXPECT_LONGS_EQUAL(5, SOn::Random(rng, 5).rows());
// }

// /* *************************************************************************
// */ TEST(SOn, HatVee) {
//   Vector6 v;
//   v << 1, 2, 3, 4, 5, 6;

//   Matrix expected2(2, 2);
//   expected2 << 0, -1, 1, 0;
//   const auto actual2 = SOn::Hat(2, v.head<1>());
//   CHECK(assert_equal(expected2, actual2));
//   CHECK(assert_equal((Vector)v.head<1>(), SOn::Vee(actual2)));

//   Matrix expected3(3, 3);
//   expected3 << 0, -3, 2,  //
//       3, 0, -1,           //
//       -2, 1, 0;
//   const auto actual3 = SOn::Hat(3, v.head<3>());
//   CHECK(assert_equal(expected3, actual3));
//   CHECK(assert_equal(skewSymmetric(1, 2, 3), actual3));
//   CHECK(assert_equal((Vector)v.head<3>(), SOn::Vee(actual3)));

//   Matrix expected4(4, 4);
//   expected4 << 0, -6, 5, -3,  //
//       6, 0, -4, 2,            //
//       -5, 4, 0, -1,           //
//       3, -2, 1, 0;
//   const auto actual4 = SOn::Hat(4, v);
//   CHECK(assert_equal(expected4, actual4));
//   CHECK(assert_equal((Vector)v, SOn::Vee(actual4)));
// }

// /* *************************************************************************
// */ TEST(SOn, RetractLocal) {
//   // If we do expmap in SO(3) subgroup, topleft should be equal to R1.
//   Vector6 v1 = (Vector(6) << 0, 0, 0, 0.01, 0, 0).finished();
//   Matrix R1 = SO3::Retract(v1.tail<3>());
//   SOn Q1 = SOn::Retract(4, v1);
//   CHECK(assert_equal(R1, Q1.block(0, 0, 3, 3), 1e-7));
//   CHECK(assert_equal(v1, SOn::Local(Q1), 1e-7));
// }

// /* *************************************************************************
// */ TEST(SOn, vec) {
//   auto x = SOn(5);
//   Vector6 v;
//   v << 1, 2, 3, 4, 5, 6;
//   x.block(0, 0, 4, 4) = SO4::Expmap(v).matrix();
//   Matrix actualH;
//   const Vector actual = x.vec(actualH);
//   boost::function<Vector(const SOn&)> h = [](const SOn& x) { return x.vec();
//   }; const Matrix H = numericalDerivative11<Vector, SOn, 10>(h, x, 1e-5);
//   CHECK(assert_equal(H, actualH));
// }

//******************************************************************************
// SO4
//******************************************************************************

TEST(SO4, Identity) {
  const SO4 R;
  EXPECT_LONGS_EQUAL(4, R.rows());
}

/* ************************************************************************* */
TEST(SO4, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<SO4>));
  BOOST_CONCEPT_ASSERT((IsManifold<SO4>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<SO4>));
}

// //******************************************************************************
// SO4 id;
// Vector6 v1 = (Vector(6) << 0.1, 0, 0, 0, 0, 0).finished();
// SO4 Q1 = SO4::Expmap(v1);
// Vector6 v2 = (Vector(6) << 0.01, 0.02, 0.03, 0.00, 0.00, 0.00).finished();
// SO4 Q2 = SO4::Expmap(v2);
// Vector6 v3 = (Vector(6) << 1, 2, 3, 4, 5, 6).finished();
// SO4 Q3 = SO4::Expmap(v3);

// //******************************************************************************
// TEST(SO4, Random) {
//   boost::mt19937 rng(42);
//   auto Q = SO4::Random(rng);
//   EXPECT_LONGS_EQUAL(4, Q.matrix().rows());
// }
// //******************************************************************************
// TEST(SO4, Expmap) {
//   // If we do exponential map in SO(3) subgroup, topleft should be equal to
//   R1. auto R1 = SO3::Expmap(v1.head<3>());
//   EXPECT((Q1.topLeft().isApprox(R1)));

//   // Same here
//   auto R2 = SO3::Expmap(v2.head<3>());
//   EXPECT((Q2.topLeft().isApprox(R2)));

//   // Check commutative subgroups
//   for (size_t i = 0; i < 6; i++) {
//     Vector6 xi = Vector6::Zero();
//     xi[i] = 2;
//     SO4 Q1 = SO4::Expmap(xi);
//     xi[i] = 3;
//     SO4 Q2 = SO4::Expmap(xi);
//     EXPECT(((Q1 * Q2).isApprox(Q2 * Q1)));
//   }
// }

// //******************************************************************************
// TEST(SO4, Cayley) {
//   CHECK(assert_equal(id.retract(v1 / 100), SO4::Expmap(v1 / 100)));
//   CHECK(assert_equal(id.retract(v2 / 100), SO4::Expmap(v2 / 100)));
// }

// //******************************************************************************
// TEST(SO4, Retract) {
//   auto v = Vector6::Zero();
//   SO4 actual = traits<SO4>::Retract(id, v);
//   EXPECT(actual.isApprox(id));
// }

// //******************************************************************************
// TEST(SO4, Local) {
//   auto v0 = Vector6::Zero();
//   Vector6 actual = traits<SO4>::Local(id, id);
//   EXPECT(assert_equal((Vector)v0, actual));
// }

// //******************************************************************************
// TEST(SO4, Invariants) {
//   EXPECT(check_group_invariants(id, id));
//   EXPECT(check_group_invariants(id, Q1));
//   EXPECT(check_group_invariants(Q2, id));
//   EXPECT(check_group_invariants(Q2, Q1));
//   EXPECT(check_group_invariants(Q1, Q2));

//   EXPECT(check_manifold_invariants(id, id));
//   EXPECT(check_manifold_invariants(id, Q1));
//   EXPECT(check_manifold_invariants(Q2, id));
//   EXPECT(check_manifold_invariants(Q2, Q1));
//   EXPECT(check_manifold_invariants(Q1, Q2));
// }

// /* *************************************************************************
// */ TEST(SO4, compose) {
//   SO4 expected = Q1 * Q2;
//   Matrix actualH1, actualH2;
//   SO4 actual = Q1.compose(Q2, actualH1, actualH2);
//   CHECK(assert_equal(expected, actual));

//   Matrix numericalH1 =
//       numericalDerivative21(testing::compose<SO4>, Q1, Q2, 1e-2);
//   CHECK(assert_equal(numericalH1, actualH1));

//   Matrix numericalH2 =
//       numericalDerivative22(testing::compose<SO4>, Q1, Q2, 1e-2);
//   CHECK(assert_equal(numericalH2, actualH2));
// }

// /* *************************************************************************
// */ TEST(SO4, vec) {
//   using Vector16 = SO4::Vector16;
//   const Vector16 expected = Eigen::Map<Vector16>(Q2.data());
//   Matrix actualH;
//   const Vector16 actual = Q2.vec(actualH);
//   CHECK(assert_equal(expected, actual));
//   boost::function<Vector16(const SO4&)> f = [](const SO4& Q) {
//     return Q.vec();
//   };
//   const Matrix numericalH = numericalDerivative11(f, Q2, 1e-5);
//   CHECK(assert_equal(numericalH, actualH));
// }

// /* *************************************************************************
// */ TEST(SO4, topLeft) {
//   const Matrix3 expected = Q3.topLeftCorner<3, 3>();
//   Matrix actualH;
//   const Matrix3 actual = Q3.topLeft(actualH);
//   CHECK(assert_equal(expected, actual));
//   boost::function<Matrix3(const SO4&)> f = [](const SO4& Q3) {
//     return Q3.topLeft();
//   };
//   const Matrix numericalH = numericalDerivative11(f, Q3, 1e-5);
//   CHECK(assert_equal(numericalH, actualH));
// }

// /* *************************************************************************
// */ TEST(SO4, stiefel) {
//   const Matrix43 expected = Q3.leftCols<3>();
//   Matrix actualH;
//   const Matrix43 actual = Q3.stiefel(actualH);
//   CHECK(assert_equal(expected, actual));
//   boost::function<Matrix43(const SO4&)> f = [](const SO4& Q3) {
//     return Q3.stiefel();
//   };
//   const Matrix numericalH = numericalDerivative11(f, Q3, 1e-5);
//   CHECK(assert_equal(numericalH, actualH));
// }

//******************************************************************************
// SO3
//******************************************************************************

TEST(SO3, Identity) {
  const SO3 R;
  EXPECT_LONGS_EQUAL(3, R.rows());
}

//******************************************************************************
TEST(SO3, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<SO3>));
  BOOST_CONCEPT_ASSERT((IsManifold<SO3>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<SO3>));
}

// //******************************************************************************
// TEST(SO3, Constructor) { SO3 q(Eigen::AngleAxisd(1, Vector3(0, 0, 1))); }

// //******************************************************************************
// SO3 id;
// Vector3 z_axis(0, 0, 1);
// SO3 R1(Eigen::AngleAxisd(0.1, z_axis));
// SO3 R2(Eigen::AngleAxisd(0.2, z_axis));

// //******************************************************************************
// TEST(SO3, Local) {
//   Vector3 expected(0, 0, 0.1);
//   Vector3 actual = traits<SO3>::Local(R1, R2);
//   EXPECT(assert_equal((Vector)expected, actual));
// }

// //******************************************************************************
// TEST(SO3, Retract) {
//   Vector3 v(0, 0, 0.1);
//   SO3 actual = traits<SO3>::Retract(R1, v);
//   EXPECT(actual.isApprox(R2));
// }

// //******************************************************************************
// TEST(SO3, Invariants) {
//   EXPECT(check_group_invariants(id, id));
//   EXPECT(check_group_invariants(id, R1));
//   EXPECT(check_group_invariants(R2, id));
//   EXPECT(check_group_invariants(R2, R1));

//   EXPECT(check_manifold_invariants(id, id));
//   EXPECT(check_manifold_invariants(id, R1));
//   EXPECT(check_manifold_invariants(R2, id));
//   EXPECT(check_manifold_invariants(R2, R1));
// }

// //******************************************************************************
// TEST(SO3, LieGroupDerivatives) {
//   CHECK_LIE_GROUP_DERIVATIVES(id, id);
//   CHECK_LIE_GROUP_DERIVATIVES(id, R2);
//   CHECK_LIE_GROUP_DERIVATIVES(R2, id);
//   CHECK_LIE_GROUP_DERIVATIVES(R2, R1);
// }

// //******************************************************************************
// TEST(SO3, ChartDerivatives) {
//   CHECK_CHART_DERIVATIVES(id, id);
//   CHECK_CHART_DERIVATIVES(id, R2);
//   CHECK_CHART_DERIVATIVES(R2, id);
//   CHECK_CHART_DERIVATIVES(R2, R1);
// }

// /* *************************************************************************
// */ namespace exmap_derivative { static const Vector3 w(0.1, 0.27, -0.2);
// }
// // Left trivialized Derivative of exp(w) wrpt w:
// // How does exp(w) change when w changes?
// // We find a y such that: exp(w) exp(y) = exp(w + dw) for dw --> 0
// // => y = log (exp(-w) * exp(w+dw))
// Vector3 testDexpL(const Vector3& dw) {
//   using exmap_derivative::w;
//   return SO3::Logmap(SO3::Expmap(-w) * SO3::Expmap(w + dw));
// }

// TEST(SO3, ExpmapDerivative) {
//   using exmap_derivative::w;
//   const Matrix actualDexpL = SO3::ExpmapDerivative(w);
//   const Matrix expectedDexpL =
//       numericalDerivative11<Vector3, Vector3>(testDexpL, Vector3::Zero(),
//       1e-2);
//   EXPECT(assert_equal(expectedDexpL, actualDexpL, 1e-7));

//   const Matrix actualDexpInvL = SO3::LogmapDerivative(w);
//   EXPECT(assert_equal(expectedDexpL.inverse(), actualDexpInvL, 1e-7));
// }

// /* *************************************************************************
// */ TEST(SO3, ExpmapDerivative2) {
//   const Vector3 theta(0.1, 0, 0.1);
//   const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
//       boost::bind(&SO3::Expmap, _1, boost::none), theta);

//   CHECK(assert_equal(Jexpected, SO3::ExpmapDerivative(theta)));
//   CHECK(assert_equal(Matrix3(Jexpected.transpose()),
//                      SO3::ExpmapDerivative(-theta)));
// }

// /* *************************************************************************
// */ TEST(SO3, ExpmapDerivative3) {
//   const Vector3 theta(10, 20, 30);
//   const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
//       boost::bind(&SO3::Expmap, _1, boost::none), theta);

//   CHECK(assert_equal(Jexpected, SO3::ExpmapDerivative(theta)));
//   CHECK(assert_equal(Matrix3(Jexpected.transpose()),
//                      SO3::ExpmapDerivative(-theta)));
// }

// /* *************************************************************************
// */ TEST(SO3, ExpmapDerivative4) {
//   // Iserles05an (Lie-group Methods) says:
//   // scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
//   // matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
//   // where A(t): R -> so(3) is a trajectory in the tangent space of SO(3)
//   // and dexp[A] is a linear map from 3*3 to 3*3 derivatives of se(3)
//   // Hence, the above matrix equation is typed: 3*3 = SO(3) * linear_map(3*3)

//   // In GTSAM, we don't work with the skew-symmetric matrices A directly, but
//   // with 3-vectors.
//   // omega is easy: d Expmap(w(t)) / dt = ExmapDerivative[w(t)] * w'(t)

//   // Let's verify the above formula.

//   auto w = [](double t) { return Vector3(2 * t, sin(t), 4 * t * t); };
//   auto w_dot = [](double t) { return Vector3(2, cos(t), 8 * t); };

//   // We define a function R
//   auto R = [w](double t) { return SO3::Expmap(w(t)); };

//   for (double t = -2.0; t < 2.0; t += 0.3) {
//     const Matrix expected = numericalDerivative11<SO3, double>(R, t);
//     const Matrix actual = SO3::ExpmapDerivative(w(t)) * w_dot(t);
//     CHECK(assert_equal(expected, actual, 1e-7));
//   }
// }

// /* *************************************************************************
// */ TEST(SO3, ExpmapDerivative5) {
//   auto w = [](double t) { return Vector3(2 * t, sin(t), 4 * t * t); };
//   auto w_dot = [](double t) { return Vector3(2, cos(t), 8 * t); };

//   // Now define R as mapping local coordinates to neighborhood around R0.
//   const SO3 R0 = SO3::Expmap(Vector3(0.1, 0.4, 0.2));
//   auto R = [R0, w](double t) { return R0.expmap(w(t)); };

//   for (double t = -2.0; t < 2.0; t += 0.3) {
//     const Matrix expected = numericalDerivative11<SO3, double>(R, t);
//     const Matrix actual = SO3::ExpmapDerivative(w(t)) * w_dot(t);
//     CHECK(assert_equal(expected, actual, 1e-7));
//   }
// }

// /* *************************************************************************
// */ TEST(SO3, ExpmapDerivative6) {
//   const Vector3 thetahat(0.1, 0, 0.1);
//   const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
//       boost::bind(&SO3::Expmap, _1, boost::none), thetahat);
//   Matrix3 Jactual;
//   SO3::Expmap(thetahat, Jactual);
//   EXPECT(assert_equal(Jexpected, Jactual));
// }

// /* *************************************************************************
// */ TEST(SO3, LogmapDerivative) {
//   const Vector3 thetahat(0.1, 0, 0.1);
//   const SO3 R = SO3::Expmap(thetahat);  // some rotation
//   const Matrix Jexpected = numericalDerivative11<Vector, SO3>(
//       boost::bind(&SO3::Logmap, _1, boost::none), R);
//   const Matrix3 Jactual = SO3::LogmapDerivative(thetahat);
//   EXPECT(assert_equal(Jexpected, Jactual));
// }

// /* *************************************************************************
// */ TEST(SO3, JacobianLogmap) {
//   const Vector3 thetahat(0.1, 0, 0.1);
//   const SO3 R = SO3::Expmap(thetahat);  // some rotation
//   const Matrix Jexpected = numericalDerivative11<Vector, SO3>(
//       boost::bind(&SO3::Logmap, _1, boost::none), R);
//   Matrix3 Jactual;
//   SO3::Logmap(R, Jactual);
//   EXPECT(assert_equal(Jexpected, Jactual));
// }

// /* *************************************************************************
// */ TEST(SO3, ApplyDexp) {
//   Matrix aH1, aH2;
//   for (bool nearZeroApprox : {true, false}) {
//     boost::function<Vector3(const Vector3&, const Vector3&)> f =
//         [=](const Vector3& omega, const Vector3& v) {
//           return so3::DexpFunctor(omega, nearZeroApprox).applyDexp(v);
//         };
//     for (Vector3 omega : {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1,
//     0),
//                           Vector3(0, 0, 1), Vector3(0.1, 0.2, 0.3)}) {
//       so3::DexpFunctor local(omega, nearZeroApprox);
//       for (Vector3 v : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
//                         Vector3(0.4, 0.3, 0.2)}) {
//         EXPECT(assert_equal(Vector3(local.dexp() * v),
//                             local.applyDexp(v, aH1, aH2)));
//         EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
//         EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
//         EXPECT(assert_equal(local.dexp(), aH2));
//       }
//     }
//   }
// }

// /* *************************************************************************
// */ TEST(SO3, ApplyInvDexp) {
//   Matrix aH1, aH2;
//   for (bool nearZeroApprox : {true, false}) {
//     boost::function<Vector3(const Vector3&, const Vector3&)> f =
//         [=](const Vector3& omega, const Vector3& v) {
//           return so3::DexpFunctor(omega, nearZeroApprox).applyInvDexp(v);
//         };
//     for (Vector3 omega : {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1,
//     0),
//                           Vector3(0, 0, 1), Vector3(0.1, 0.2, 0.3)}) {
//       so3::DexpFunctor local(omega, nearZeroApprox);
//       Matrix invDexp = local.dexp().inverse();
//       for (Vector3 v : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
//                         Vector3(0.4, 0.3, 0.2)}) {
//         EXPECT(assert_equal(Vector3(invDexp * v),
//                             local.applyInvDexp(v, aH1, aH2)));
//         EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
//         EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
//         EXPECT(assert_equal(invDexp, aH2));
//       }
//     }
//   }
// }

// /* *************************************************************************
// */ TEST(SO3, vec) {
//   const Vector9 expected = Eigen::Map<Vector9>(R2.data());
//   Matrix actualH;
//   const Vector9 actual = R2.vec(actualH);
//   CHECK(assert_equal(expected, actual));
//   boost::function<Vector9(const SO3&)> f = [](const SO3& Q) {
//     return Q.vec();
//   };
//   const Matrix numericalH = numericalDerivative11(f, R2, 1e-5);
//   CHECK(assert_equal(numericalH, actualH));
// }

// //******************************************************************************
// TEST(Matrix, compose) {
//   Matrix3 M;
//   M << 1, 2, 3, 4, 5, 6, 7, 8, 9;
//   SO3 R = SO3::Expmap(Vector3(1, 2, 3));
//   const Matrix3 expected = M * R.matrix();
//   Matrix actualH;
//   const Matrix3 actual = so3::compose(M, R, actualH);
//   CHECK(assert_equal(expected, actual));
//   boost::function<Matrix3(const Matrix3&)> f = [R](const Matrix3& M) {
//     return so3::compose(M, R);
//   };
//   Matrix numericalH = numericalDerivative11(f, M, 1e-2);
//   CHECK(assert_equal(numericalH, actualH));
// }

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
