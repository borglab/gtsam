/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file   testExtendedPose3.cpp
 * @brief  Unit tests for ExtendedPose3 class
 * @author Martin Brossard
 */

#include <gtsam/base/lieProxies.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/ExtendedPose3.h>

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(ExtendedPose3)
GTSAM_CONCEPT_LIE_INST(ExtendedPose3)

static const Point3 V(3, 0.4, -2.2);
static const Point3 P(0.2, 0.7, -2);
static const Rot3 R = Rot3::Rodrigues(0.3, 0, 0);
static const Point3 V2(-6.5, 3.5, 6.2);
static const Point3 P2(3.5, -8.2, 4.2);
static const ExtendedPose3 T(R, V2, P2);
static const ExtendedPose3 T2(Rot3::Rodrigues(0.3, 0.2, 0.1), V2, P2);
static const ExtendedPose3 T3(Rot3::Rodrigues(-90, 0, 0), Point3(5, 6, 7), Point3(1, 2, 3));

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/* ************************************************************************* */
TEST(ExtendedPose3, equals) {
  ExtendedPose3 pose2 = T3;
  EXPECT(T3.equals(pose2));
  ExtendedPose3 origin;
  EXPECT(!T3.equals(origin));
}

/* ************************************************************************* */
#ifndef GTSAM_POSE3_EXPMAP
TEST(ExtendedPose3, retract_first_order) {
  ExtendedPose3 id;
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  EXPECT(assert_equal(ExtendedPose3(R, Vector3(0, 0, 0), Point3(0, 0, 0)), id.retract(xi), 1e-2));
  xi(3) = 3;
  xi(4) = 0.4;
  xi(5) = -2.2;
  xi(6) = 0.2;
  xi(7) = 0.7;
  xi(8) = -2;
  EXPECT(assert_equal(ExtendedPose3(R, V, P), id.retract(v), 1e-2));
}
#endif
/* ************************************************************************* */
TEST(ExtendedPose3, retract_expmap) {
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  ExtendedPose3 pose = ExtendedPose3::Expmap(xi);
  EXPECT(assert_equal(ExtendedPose3(R, Point3(0, 0, 0), Point3(0, 0, 0)), pose, 1e-2));
  EXPECT(assert_equal(xi, ExtendedPose3::Logmap(pose), 1e-2));
}

/* ************************************************************************* */
TEST(ExtendedPose3, expmap_a_full) {
  ExtendedPose3 id;
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  EXPECT(assert_equal(expmap_default<ExtendedPose3>(id, xi), ExtendedPose3(R, Vector3(0, 0, 0), Point3(0, 0, 0))));
  xi(3) = -0.2;
  xi(4) = -0.394742;
  xi(5) = 2.08998;
  xi(6) = 0.2;
  xi(7) = 0.394742;
  xi(8) = -2.08998;
  EXPECT(assert_equal(ExtendedPose3(R, -P, P), expmap_default<ExtendedPose3>(id, xi), 1e-5));
}

/* ************************************************************************* */
TEST(ExtendedPose3, expmap_a_full2) {
  ExtendedPose3 id;
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  EXPECT(assert_equal(expmap_default<ExtendedPose3>(id, xi), ExtendedPose3(R, Point3(0, 0, 0), Point3(0, 0, 0))));
  xi(3) = -0.2;
  xi(4) = -0.394742;
  xi(5) = 2.08998;
  xi(6) = 0.2;
  xi(7) = 0.394742;
  xi(8) = -2.08998;
  EXPECT(assert_equal(ExtendedPose3(R, -P, P), expmap_default<ExtendedPose3>(id, xi), 1e-5));
}

/* ************************************************************************* */
TEST(ExtendedPose3, expmap_b) {
  ExtendedPose3 p1(Rot3(), Vector3(-100, 0, 0), Point3(100, 0, 0));
  ExtendedPose3 p2 = p1.retract((Vector(9) << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  ExtendedPose3 expected(Rot3::Rodrigues(0.0, 0.0, 0.1), Point3(-100.0, 0.0, 0.0), Point3(100.0, 0.0, 0.0));
  EXPECT(assert_equal(expected, p2, 1e-2));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screwExtendedPose3 {
double a = 0.3, c = cos(a), s = sin(a), w = 0.3;
Vector xi = (Vector(9) << 0.0, 0.0, w, w, 0.0, 1.0, w, 0.0, 1.0).finished();
Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
Point3 expectedV(0.29552, 0.0446635, 1);
Point3 expectedP(0.29552, 0.0446635, 1);
ExtendedPose3 expected(expectedR, expectedV, expectedP);
}  // namespace screwExtendedPose3

/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(ExtendedPose3, expmap_c_full) {
  EXPECT(assert_equal(screwExtendedPose3::expected, expm<ExtendedPose3>(screwExtendedPose3::xi), 1e-6));
  EXPECT(assert_equal(screwExtendedPose3::expected, ExtendedPose3::Expmap(screwExtendedPose3::xi), 1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(ExtendedPose3, Adjoint_full) {
  ExtendedPose3 expected = T * ExtendedPose3::Expmap(screwExtendedPose3::xi) * T.inverse();
  Vector xiprime = T.Adjoint(screwExtendedPose3::xi);
  EXPECT(assert_equal(expected, ExtendedPose3::Expmap(xiprime), 1e-6));

  ExtendedPose3 expected2 = T2 * ExtendedPose3::Expmap(screwExtendedPose3::xi) * T2.inverse();
  Vector xiprime2 = T2.Adjoint(screwExtendedPose3::xi);
  EXPECT(assert_equal(expected2, ExtendedPose3::Expmap(xiprime2), 1e-6));

  ExtendedPose3 expected3 = T3 * ExtendedPose3::Expmap(screwExtendedPose3::xi) * T3.inverse();
  Vector xiprime3 = T3.Adjoint(screwExtendedPose3::xi);
  EXPECT(assert_equal(expected3, ExtendedPose3::Expmap(xiprime3), 1e-6));
}

/* ************************************************************************* */
// assert that T*wedge(xi)*T^-1 is equal to wedge(Ad_T(xi))
TEST(ExtendedPose3, Adjoint_hat) {
  auto hat = [](const Vector& xi) { return ::wedge<ExtendedPose3>(xi); };
  Matrix5 expected = T.matrix() * hat(screwExtendedPose3::xi) * T.matrix().inverse();
  Matrix5 xiprime = hat(T.Adjoint(screwExtendedPose3::xi));

  EXPECT(assert_equal(expected, xiprime, 1e-6));

  Matrix5 expected2 = T2.matrix() * hat(screwExtendedPose3::xi) * T2.matrix().inverse();
  Matrix5 xiprime2 = hat(T2.Adjoint(screwExtendedPose3::xi));
  EXPECT(assert_equal(expected2, xiprime2, 1e-6));

  Matrix5 expected3 = T3.matrix() * hat(screwExtendedPose3::xi) * T3.matrix().inverse();

  Matrix5 xiprime3 = hat(T3.Adjoint(screwExtendedPose3::xi));
  EXPECT(assert_equal(expected3, xiprime3, 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, expmaps_galore_full) {
  Vector xi;
  ExtendedPose3 actual;
  xi = (Vector(9) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished();
  actual = ExtendedPose3::Expmap(xi);
  EXPECT(assert_equal(expm<ExtendedPose3>(xi), actual, 1e-6));
  EXPECT(assert_equal(xi, ExtendedPose3::Logmap(actual), 1e-6));

  xi = (Vector(9) << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, -0.7, -0.8, -0.9).finished();
  for (double theta = 1.0; 0.3 * theta <= M_PI; theta *= 2) {
    Vector txi = xi * theta;
    actual = ExtendedPose3::Expmap(txi);
    EXPECT(assert_equal(expm<ExtendedPose3>(txi, 30), actual, 1e-6));
    Vector log = ExtendedPose3::Logmap(actual);
    EXPECT(assert_equal(actual, ExtendedPose3::Expmap(log), 1e-6));
    EXPECT(assert_equal(txi, log, 1e-6));  // not true once wraps
  }

  // Works with large v as well, but expm needs 10 iterations!
  xi = (Vector(9) << 0.2, 0.3, -0.8, 100.0, 120.0, -60.0, 12, 14, 45).finished();
  actual = ExtendedPose3::Expmap(xi);
  EXPECT(assert_equal(expm<ExtendedPose3>(xi, 10), actual, 1e-5));
  EXPECT(assert_equal(xi, ExtendedPose3::Logmap(actual), 1e-9));
}

/* ************************************************************************* */
// Check position and its pushforward

TEST(ExtendedPose3, position) {
  Matrix actualH;
  EXPECT(assert_equal(Point3(3.5, -8.2, 4.2), T.position(actualH), 1e-8));
  std::function<Point3(const ExtendedPose3&)> f = [](const ExtendedPose3& T) { return T.position(); };
  Matrix numericalH = numericalDerivative11<Point3, ExtendedPose3>(f, T);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
// Check rotation and its pushforward
TEST(ExtendedPose3, rotation) {
  Matrix actualH;
  EXPECT(assert_equal(R, T.rotation(actualH), 1e-8));

  std::function<Rot3(const ExtendedPose3&)> f = [](const ExtendedPose3& T) { return T.rotation(); };
  Matrix numericalH = numericalDerivative11<Rot3, ExtendedPose3>(f, T);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
// Check velocity and its pushforward
TEST(ExtendedPose3, velocity) {
  Matrix actualH;
  EXPECT(assert_equal(Point3(-6.5, 3.5, 6.2), T.velocity(actualH), 1e-8));

  std::function<Point3(const ExtendedPose3&)> f = [](const ExtendedPose3& T) { return T.velocity(); };
  Matrix numericalH = numericalDerivative11<Point3, ExtendedPose3>(f, T);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Adjoint_compose_full) {
  // To debug derivatives of compose, assert that
  // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
  const ExtendedPose3& T1 = T;
  Vector x = (Vector(9) << 0.1, 0.1, 0.1, 0.4, 0.2, 0.8, 0.4, 0.2, 0.8).finished();
  ExtendedPose3 expected = T1 * ExtendedPose3::Expmap(x) * T2;
  Vector y = T2.inverse().Adjoint(x);
  ExtendedPose3 actual = T1 * T2 * ExtendedPose3::Expmap(y);
  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* ************************************************************************* */
// Check compose and its pushforward
// NOTE: testing::compose<ExtendedPose3>(t1,t2) = t1.compose(t2)  (see lieProxies.h)
TEST(ExtendedPose3, compose) {
  Matrix actual = (T2 * T2).matrix();

  Matrix expected = T2.matrix() * T2.matrix();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T2.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21(testing::compose<ExtendedPose3>, T2, T2);

  EXPECT(assert_equal(numericalH1, actualDcompose1, 5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::compose<ExtendedPose3>, T2, T2);
  EXPECT(assert_equal(numericalH2, actualDcompose2, 1e-4));
}

/* ************************************************************************* */
// Check compose and its pushforward, another case
TEST(ExtendedPose3, compose2) {
  const ExtendedPose3& T1 = T;
  Matrix actual = (T1 * T2).matrix();
  Matrix expected = T1.matrix() * T2.matrix();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T1.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21(testing::compose<ExtendedPose3>, T1, T2);
  EXPECT(assert_equal(numericalH1, actualDcompose1, 5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::compose<ExtendedPose3>, T1, T2);
  EXPECT(assert_equal(numericalH2, actualDcompose2, 1e-5));
}

/* ************************************************************************* */
TEST(ExtendedPose3, inverse) {
  Matrix actualDinverse;
  Matrix actual = T.inverse(actualDinverse).matrix();
  Matrix expected = T.matrix().inverse();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix numericalH = numericalDerivative11(testing::inverse<ExtendedPose3>, T);
  EXPECT(assert_equal(numericalH, actualDinverse, 5e-3));
  EXPECT(assert_equal(-T.AdjointMap(), actualDinverse, 5e-3));
}

/* ************************************************************************* */
TEST(ExtendedPose3, inverseDerivatives2) {
  Rot3 R = Rot3::Rodrigues(0.3, 0.4, -0.5);
  Vector3 v(3.5, -8.2, 4.2);
  Point3 p(3.5, -8.2, 4.2);
  ExtendedPose3 T(R, v, p);

  Matrix numericalH = numericalDerivative11(testing::inverse<ExtendedPose3>, T);
  Matrix actualDinverse;
  T.inverse(actualDinverse);
  EXPECT(assert_equal(numericalH, actualDinverse, 5e-3));
  EXPECT(assert_equal(-T.AdjointMap(), actualDinverse, 5e-3));
}

/* ************************************************************************* */
TEST(ExtendedPose3, compose_inverse) {
  Matrix actual = (T * T.inverse()).matrix();
  Matrix expected = I_5x5;
  EXPECT(assert_equal(actual, expected, 1e-8));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Retract_LocalCoordinates) {
  Vector9 d;
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  d /= 10;
  const Rot3 R = Rot3::Retract(d.head<3>());
  ExtendedPose3 t = ExtendedPose3::Retract(d);
  EXPECT(assert_equal(d, ExtendedPose3::LocalCoordinates(t)));
}
/* ************************************************************************* */
TEST(ExtendedPose3, retract_localCoordinates) {
  Vector9 d12;
  d12 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  d12 /= 10;
  ExtendedPose3 t1 = T, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}
/* ************************************************************************* */
TEST(ExtendedPose3, expmap_logmap) {
  Vector d12 = Vector9::Constant(0.1);
  ExtendedPose3 t1 = T, t2 = t1.expmap(d12);
  EXPECT(assert_equal(d12, t1.logmap(t2)));
}

/* ************************************************************************* */
TEST(ExtendedPose3, retract_localCoordinates2) {
  ExtendedPose3 t1 = T;
  ExtendedPose3 t2 = T3;
  ExtendedPose3 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
  EXPECT(assert_equal(d12, -d21));
}
/* ************************************************************************* */
TEST(ExtendedPose3, manifold_expmap) {
  ExtendedPose3 t1 = T;
  ExtendedPose3 t2 = T3;
  ExtendedPose3 origin;
  Vector d12 = t1.logmap(t2);
  EXPECT(assert_equal(t2, t1.expmap(d12)));
  Vector d21 = t2.logmap(t1);
  EXPECT(assert_equal(t1, t2.expmap(d21)));

  // Check that log(t1,t2)=-log(t2,t1)
  EXPECT(assert_equal(d12, -d21));
}

/* ************************************************************************* */
TEST(ExtendedPose3, subgroups) {
  // Frank - Below only works for correct "Agrawal06iros style expmap
  // lines in canonical coordinates correspond to Abelian subgroups in SE(3)
  Vector d = (Vector(9) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished();
  // exp(-d)=inverse(exp(d))
  EXPECT(assert_equal(ExtendedPose3::Expmap(-d), ExtendedPose3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  ExtendedPose3 T2 = ExtendedPose3::Expmap(2 * d);
  ExtendedPose3 T3 = ExtendedPose3::Expmap(3 * d);
  ExtendedPose3 T5 = ExtendedPose3::Expmap(5 * d);
  EXPECT(assert_equal(T5, T2 * T3));
  EXPECT(assert_equal(T5, T3 * T2));
}

/* ************************************************************************* */
TEST(ExtendedPose3, between) {
  ExtendedPose3 expected = T2.inverse() * T3;
  Matrix actualDBetween1, actualDBetween2;
  ExtendedPose3 actual = T2.between(T3, actualDBetween1, actualDBetween2);
  EXPECT(assert_equal(expected, actual));

  Matrix numericalH1 = numericalDerivative21(testing::between<ExtendedPose3>, T2, T3);
  EXPECT(assert_equal(numericalH1, actualDBetween1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::between<ExtendedPose3>, T2, T3);
  EXPECT(assert_equal(numericalH2, actualDBetween2, 1e-5));
}

/* ************************************************************************* */
TEST(ExtendedPose3, adjointMap) {
  Matrix res = ExtendedPose3::adjointMap(screwExtendedPose3::xi);
  Matrix wh = skewSymmetric(screwExtendedPose3::xi(0), screwExtendedPose3::xi(1), screwExtendedPose3::xi(2));
  Matrix vh = skewSymmetric(screwExtendedPose3::xi(3), screwExtendedPose3::xi(4), screwExtendedPose3::xi(5));
  Matrix rh = skewSymmetric(screwExtendedPose3::xi(6), screwExtendedPose3::xi(7), screwExtendedPose3::xi(8));
  Matrix9 expected;
  expected << wh, Z_3x3, Z_3x3, vh, wh, Z_3x3, rh, Z_3x3, wh;
  EXPECT(assert_equal(expected, res, 1e-5));
}

/* ************************************************************************* */

TEST(ExtendedPose3, ExpmapDerivative1) {
  Matrix9 actualH;
  Vector9 w;
  w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ExtendedPose3::Expmap(w, actualH);

  std::function<ExtendedPose3(const Vector9&)> f = [](const Vector9& w) { return ExtendedPose3::Expmap(w); };
  Matrix expectedH =
      numericalDerivative21<ExtendedPose3, Vector9, OptionalJacobian<9, 9> >(&ExtendedPose3::Expmap, w, {});
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
TEST(ExtendedPose3, LogmapDerivative) {
  Matrix9 actualH;
  Vector9 w;
  w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ExtendedPose3 p = ExtendedPose3::Expmap(w);
  EXPECT(assert_equal(w, ExtendedPose3::Logmap(p, actualH), 1e-5));

  std::function<Vector9(const ExtendedPose3&)> f = [](const ExtendedPose3& p) { return ExtendedPose3::Logmap(p); };
  Matrix expectedH =
      numericalDerivative21<Vector9, ExtendedPose3, OptionalJacobian<9, 9> >(&ExtendedPose3::Logmap, p, {});
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
TEST(ExtendedPose3, stream) {
  ExtendedPose3 T;
  std::ostringstream os;
  os << T;
  std::string expected = "[\n"
      "\t1, 0, 0;\n"
      "\t0, 1, 0;\n"
      "\t0, 0, 1\n"
      "]v:[0, 0, 0];\n"
      "p:[0, 0, 0];\n"
      "";
  EXPECT(os.str() == expected);
}

//******************************************************************************
TEST(ExtendedPose3, Invariants) {
  ExtendedPose3 id;

  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, T3));
  EXPECT(check_group_invariants(T2, id));
  EXPECT(check_group_invariants(T2, T3));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, T3));
  EXPECT(check_manifold_invariants(T2, id));
  EXPECT(check_manifold_invariants(T2, T3));
}

//******************************************************************************
TEST(ExtendedPose3, LieGroupDerivatives) {
  ExtendedPose3 id;

  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2, id);
  CHECK_LIE_GROUP_DERIVATIVES(T2, T3);
}

//******************************************************************************
TEST(ExtendedPose3, ChartDerivatives) {
  ExtendedPose3 id;
  if (ROT3_DEFAULT_COORDINATES_MODE == Rot3::EXPMAP) {
    CHECK_CHART_DERIVATIVES(id, id);
    //    CHECK_CHART_DERIVATIVES(id,T2);
    //    CHECK_CHART_DERIVATIVES(T2,id);
    //    CHECK_CHART_DERIVATIVES(T2,T3);
  }
}

/* ************************************************************************* */
TEST(ExtendedPose3, interpolate) {
  EXPECT(assert_equal(T2, interpolate(T2, T3, 0.0)));
  EXPECT(assert_equal(T3, interpolate(T2, T3, 1.0)));
}

/* ************************************************************************* */

TEST(ExtendedPose3, Create) {
  Matrix93 actualH1, actualH2, actualH3;
  ExtendedPose3 actual = ExtendedPose3::Create(R, V2, P2, actualH1, actualH2, actualH3);
  EXPECT(assert_equal(T, actual));
  std::function<ExtendedPose3(Rot3, Point3, Point3)> create = [](Rot3 rot, Point3 p1, Point3 p2) {
    return ExtendedPose3::Create(rot, p1, p2);
  };
  EXPECT(assert_equal(numericalDerivative31<ExtendedPose3, Rot3, Point3, Point3>(create, R, V2, P2), actualH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative32<ExtendedPose3, Rot3, Point3, Point3>(create, R, V2, P2), actualH2, 1e-9));
  EXPECT(assert_equal(numericalDerivative33<ExtendedPose3, Rot3, Point3, Point3>(create, R, V2, P2), actualH3, 1e-9));
}

/* ************************************************************************* */
TEST(ExtendedPose3, print) {
  std::stringstream redirectStream;
  std::streambuf* ssbuf = redirectStream.rdbuf();
  std::streambuf* oldbuf = std::cout.rdbuf();
  // redirect cout to redirectStream
  std::cout.rdbuf(ssbuf);

  ExtendedPose3 pose(Rot3::Identity(), Vector3(1, 2, 3), Point3(1, 2, 3));
  // output is captured to redirectStream
  pose.print();

  // Generate the expected output
  std::stringstream expected;
  Vector3 velocity(1, 2, 3);
  Point3 position(1, 2, 3);

  expected << "v:" << velocity.x() << "\n" << velocity.y() << "\n" << velocity.z() << ";\np:" << position.x()
           << "\n" << position.y() << "\n" << position.z() << ";\n";

  // reset cout to the original stream
  std::cout.rdbuf(oldbuf);

  // Get substring corresponding to position part
  std::string actual = redirectStream.str().substr(37);
  CHECK(expected.str() == actual);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
