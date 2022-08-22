/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSimilarity3.cpp
 * @brief  Unit tests for Similarity3 class
 * @author Paul Drews
 * @author Zhaoyang Lv
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <functional>

using namespace std::placeholders;
using namespace gtsam;
using namespace std;
using symbol_shorthand::X;

GTSAM_CONCEPT_TESTABLE_INST(Similarity3)

static const Point3 P(0.2, 0.7, -2);
static const Rot3 R = Rot3::Rodrigues(0.3, 0, 0);
static const double s = 4;
static const Similarity3 id;
static const Similarity3 T1(R, Point3(3.5, -8.2, 4.2), 1);
static const Similarity3 T2(Rot3::Rodrigues(0.3, 0.2, 0.1),
    Point3(3.5, -8.2, 4.2), 1);
static const Similarity3 T3(Rot3::Rodrigues(-90, 0, 0), Point3(1, 2, 3), 1);
static const Similarity3 T4(R, P, s);
static const Similarity3 T5(R, P, 10);
static const Similarity3 T6(Rot3(), Point3(1, 1, 0), 2); // Simpler transform

const double degree = M_PI / 180;

//******************************************************************************
TEST(Similarity3, Concepts) {
  BOOST_CONCEPT_ASSERT((IsGroup<Similarity3 >));
  BOOST_CONCEPT_ASSERT((IsManifold<Similarity3 >));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Similarity3 >));
}

//******************************************************************************
TEST(Similarity3, Constructors) {
  Similarity3 sim3_Construct1;
  Similarity3 sim3_Construct2(s);
  Similarity3 sim3_Construct3(R, P, s);
  Similarity3 sim4_Construct4(R.matrix(), P, s);
}

//******************************************************************************
TEST(Similarity3, Getters) {
  Similarity3 sim3_default;
  EXPECT(assert_equal(Rot3(), sim3_default.rotation()));
  EXPECT(assert_equal(Point3(0,0,0), sim3_default.translation()));
  EXPECT_DOUBLES_EQUAL(1.0, sim3_default.scale(), 1e-9);

  Similarity3 sim3(Rot3::Ypr(1, 2, 3), Point3(4, 5, 6), 7);
  EXPECT(assert_equal(Rot3::Ypr(1, 2, 3), sim3.rotation()));
  EXPECT(assert_equal(Point3(4, 5, 6), sim3.translation()));
  EXPECT_DOUBLES_EQUAL(7.0, sim3.scale(), 1e-9);
}

//******************************************************************************
TEST(Similarity3, AdjointMap) {
  const Matrix4 T = T2.matrix();
  // Check Ad with actual definition
  Vector7 delta;
  delta << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  Matrix4 W = Similarity3::wedge(delta);
  Matrix4 TW = Similarity3::wedge(T2.AdjointMap() * delta);
  EXPECT(assert_equal(TW, Matrix4(T * W * T.inverse()), 1e-9));
}

//******************************************************************************
TEST(Similarity3, inverse) {
  Similarity3 sim3(Rot3::Ypr(1, 2, 3).inverse(), Point3(4, 5, 6), 7);
  Matrix3 Re; // some values from matlab
  Re << -0.2248, 0.9024, -0.3676, -0.3502, -0.4269, -0.8337, -0.9093, -0.0587, 0.4120;
  Vector3 te(-9.8472, 59.7640, 10.2125);
  Similarity3 expected(Re, te, 1.0 / 7.0);
  EXPECT(assert_equal(expected, sim3.inverse(), 1e-4));
  EXPECT(assert_equal(sim3, sim3.inverse().inverse(), 1e-8));

  // test lie group inverse
  Matrix H1, H2;
  EXPECT(assert_equal(expected, sim3.inverse(H1), 1e-4));
  EXPECT(assert_equal(sim3, sim3.inverse().inverse(H2), 1e-8));
}

//******************************************************************************
TEST(Similarity3, Multiplication) {
  Similarity3 test1(Rot3::Ypr(1, 2, 3).inverse(), Point3(4, 5, 6), 7);
  Similarity3 test2(Rot3::Ypr(1, 2, 3).inverse(), Point3(8, 9, 10), 11);
  Matrix3 re;
  re << 0.0688, 0.9863, -0.1496, -0.5665, -0.0848, -0.8197, -0.8211, 0.1412, 0.5530;
  Vector3 te(-13.6797, 3.2441, -5.7794);
  Similarity3 expected(re, te, 77);
  EXPECT(assert_equal(expected, test1 * test2, 1e-2));
}

//******************************************************************************
TEST(Similarity3, Manifold) {
  EXPECT_LONGS_EQUAL(7, Similarity3::Dim());
  Vector z = Vector7::Zero();
  Similarity3 sim;
  EXPECT(sim.retract(z) == sim);

  Vector7 v = Vector7::Zero();
  v(6) = 2;
  Similarity3 sim2;
  EXPECT(sim2.retract(z) == sim2);

  EXPECT(assert_equal(z, sim2.localCoordinates(sim)));

  Similarity3 sim3 = Similarity3(Rot3(), Point3(1, 2, 3), 1);
  Vector v3(7);
  v3 << 0, 0, 0, 1, 2, 3, 0;
  EXPECT(assert_equal(v3, sim2.localCoordinates(sim3)));

  Similarity3 other = Similarity3(Rot3::Ypr(0.1, 0.2, 0.3), Point3(4, 5, 6), 1);

  Vector vlocal = sim.localCoordinates(other);

  EXPECT(assert_equal(sim.retract(vlocal), other, 1e-2));

  Similarity3 other2 = Similarity3(Rot3::Ypr(0.3, 0, 0), Point3(4, 5, 6), 1);
  Rot3 R = Rot3::Rodrigues(0.3, 0, 0);

  Vector vlocal2 = sim.localCoordinates(other2);

  EXPECT(assert_equal(sim.retract(vlocal2), other2, 1e-2));

  // TODO add unit tests for retract and localCoordinates
}

//******************************************************************************
TEST( Similarity3, retract_first_order) {
  Similarity3 id;
  Vector v = Z_7x1;
  v(0) = 0.3;
  EXPECT(assert_equal(Similarity3(R, Point3(0,0,0), 1), id.retract(v), 1e-2));
//  v(3) = 0.2;
//  v(4) = 0.7;
//  v(5) = -2;
//  EXPECT(assert_equal(Similarity3(R, P, 1), id.retract(v), 1e-2));
}

//******************************************************************************
TEST(Similarity3, localCoordinates_first_order) {
  Vector7 d12 = Vector7::Constant(0.1);
  d12(6) = 1.0;
  Similarity3 t1 = T1, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}

//******************************************************************************
TEST(Similarity3, manifold_first_order) {
  Similarity3 t1 = T1;
  Similarity3 t2 = T3;
  Similarity3 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
}

//******************************************************************************
// Return as a 4*4 Matrix
TEST(Similarity3, Matrix) {
  Matrix4 expected;
  expected << 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0.5;
  Matrix4 actual = T6.matrix();
  EXPECT(assert_equal(expected, actual));
}

//*****************************************************************************
// Exponential and log maps
TEST(Similarity3, ExpLogMap) {
  Vector7 delta;
  delta << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  Vector7 actual = Similarity3::Logmap(Similarity3::Expmap(delta));
  EXPECT(assert_equal(delta, actual));

  Vector7 zeros;
  zeros << 0, 0, 0, 0, 0, 0, 0;
  Vector7 logIdentity = Similarity3::Logmap(Similarity3::Identity());
  EXPECT(assert_equal(zeros, logIdentity));

  Similarity3 expZero = Similarity3::Expmap(zeros);
  Similarity3 ident = Similarity3::Identity();
  EXPECT(assert_equal(expZero, ident));

  // Compare to matrix exponential, using expm in Lie.h
  EXPECT(
      assert_equal(expm<Similarity3>(delta), Similarity3::Expmap(delta), 1e-3));
}

//******************************************************************************
// Group action on Point3 (with simpler transform)
TEST(Similarity3, GroupAction) {
  EXPECT(assert_equal(Point3(2, 2, 0), T6 * Point3(0, 0, 0)));
  EXPECT(assert_equal(Point3(4, 2, 0), T6 * Point3(1, 0, 0)));

  // Test group action on R^4 via matrix representation
  Vector4 qh;
  qh << 1, 0, 0, 1;
  Vector4 ph;
  ph << 2, 1, 0, 0.5; // equivalent to Point3(4, 2, 0)
  EXPECT(assert_equal((Vector )ph, T6.matrix() * qh));

  // Test some more...
  Point3 pa = Point3(1, 0, 0);
  Similarity3 Ta(Rot3(), Point3(1, 2, 3), 1.0);
  Similarity3 Tb(Rot3(), Point3(1, 2, 3), 2.0);
  EXPECT(assert_equal(Point3(2, 2, 3), Ta.transformFrom(pa)));
  EXPECT(assert_equal(Point3(4, 4, 6), Tb.transformFrom(pa)));

  Similarity3 Tc(Rot3::Rz(M_PI / 2.0), Point3(1, 2, 3), 1.0);
  Similarity3 Td(Rot3::Rz(M_PI / 2.0), Point3(1, 2, 3), 2.0);
  EXPECT(assert_equal(Point3(1, 3, 3), Tc.transformFrom(pa)));
  EXPECT(assert_equal(Point3(2, 6, 6), Td.transformFrom(pa)));

  // Test derivative
  // Use lambda to resolve overloaded method
  std::function<Point3(const Similarity3&, const Point3&)>
      f = [](const Similarity3& S, const Point3& p){ return S.transformFrom(p); };

  Point3 q(1, 2, 3);
  for (const auto& T : { T1, T2, T3, T4, T5, T6 }) {
    Point3 q(1, 0, 0);
    Matrix H1 = numericalDerivative21<Point3, Similarity3, Point3>(f, T, q);
    Matrix H2 = numericalDerivative22<Point3, Similarity3, Point3>(f, T, q);
    Matrix actualH1, actualH2;
    T.transformFrom(q, actualH1, actualH2);
    EXPECT(assert_equal(H1, actualH1));
    EXPECT(assert_equal(H2, actualH2));
  }
}

//******************************************************************************
// Group action on Pose3
// Estimate Sim(3) object "aSb" from pose pairs {(aTi, bTi)}
// In the example below, let the "a" frame be the "world" frame below,
// and let the "b" frame be the "egovehicle" frame.
// Suppose within the egovehicle frame, we know the poses of two objects "o1" and "o2"
TEST(Similarity3, GroupActionPose3) {
  // Suppose we know the pose of the egovehicle in the world frame
  Similarity3 wSe(Rot3::Ry(180 * degree), Point3(2, 3, 5), 2.0);

  // Create source poses (two objects o1 and o2 living in the egovehicle "e" frame)
  // Suppose they are 3d cuboids detected by onboard sensor, in the egovehicle frame
  Pose3 eTo1(Rot3(), Point3(0, 0, 0));
  Pose3 eTo2(Rot3(-1, 0, 0, 0, -1, 0, 0, 0, 1), Point3(4, 0, 0));

  // Create destination poses (two objects now living in the world/city "w" frame)
  // Desired to place the objects into the world frame for tracking
  Pose3 expected_wTo1(Rot3(-1, 0, 0, 0, 1, 0, 0, 0, -1), Point3(4, 6, 10));
  Pose3 expected_wTo2(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(-4, 6, 10));

  // objects now live in the world frame, instead of in the egovehicle frame
  EXPECT(assert_equal(expected_wTo1, wSe.transformFrom(eTo1)));
  EXPECT(assert_equal(expected_wTo2, wSe.transformFrom(eTo2)));
}

// Test left group action compatibility.
// cSa*Ta = cSb*bSa*Ta
TEST(Similarity3, GroupActionPose3_Compatibility) {
  Similarity3 bSa(Rot3::Ry(180 * degree), Point3(2, 3, 5), 2.0);
  Similarity3 cSb(Rot3::Ry(90 * degree), Point3(-10, -4, 0), 3.0);
  Similarity3 cSa(Rot3::Ry(270 * degree), Point3(0, 1, -2), 6.0);

  // Create poses
  Pose3 Ta1(Rot3(), Point3(0, 0, 0));
  Pose3 Ta2(Rot3(-1, 0, 0, 0, -1, 0, 0, 0, 1), Point3(4, 0, 0));
  Pose3 Tb1(Rot3(-1, 0, 0, 0, 1, 0, 0, 0, -1), Point3(4, 6, 10));
  Pose3 Tb2(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(-4, 6, 10));
  Pose3 Tc1(Rot3(0, 0, -1, 0, 1, 0, 1, 0, 0), Point3(0, 6, -12));
  Pose3 Tc2(Rot3(0, 0, -1, 0, -1, 0, -1, 0, 0), Point3(0, 6, 12));

  EXPECT(assert_equal(Tc1, cSb.transformFrom(Tb1)));
  EXPECT(assert_equal(Tc2, cSb.transformFrom(Tb2)));

  EXPECT(assert_equal(cSa.transformFrom(Ta1), cSb.transformFrom(Tb1)));
  EXPECT(assert_equal(cSa.transformFrom(Ta2), cSb.transformFrom(Tb2)));
}

//******************************************************************************
// Align with Point3 Pairs
TEST(Similarity3, AlignPoint3_1) {
  Similarity3 expected_aSb(Rot3::Rz(-90 * degree), Point3(3, 4, 5), 2.0);

  Point3 b1(0, 0, 0), b2(3, 0, 0), b3(3, 0, 4);

  Point3Pair ab1(make_pair(expected_aSb.transformFrom(b1), b1));
  Point3Pair ab2(make_pair(expected_aSb.transformFrom(b2), b2));
  Point3Pair ab3(make_pair(expected_aSb.transformFrom(b3), b3));

  vector<Point3Pair> correspondences{ab1, ab2, ab3};

  Similarity3 actual_aSb = Similarity3::Align(correspondences);
  EXPECT(assert_equal(expected_aSb, actual_aSb));
}

TEST(Similarity3, AlignPoint3_2) {
  Similarity3 expected_aSb(Rot3(), Point3(10, 10, 0), 1.0);

  Point3 b1(0, 0, 0), b2(20, 10, 0), b3(10, 20, 0);

  Point3Pair ab1(make_pair(expected_aSb.transformFrom(b1), b1));
  Point3Pair ab2(make_pair(expected_aSb.transformFrom(b2), b2));
  Point3Pair ab3(make_pair(expected_aSb.transformFrom(b3), b3));

  vector<Point3Pair> correspondences{ab1, ab2, ab3};

  Similarity3 actual_aSb = Similarity3::Align(correspondences);
  EXPECT(assert_equal(expected_aSb, actual_aSb));
}

TEST(Similarity3, AlignPoint3_3) {
  Similarity3 expected_aSb(Rot3::RzRyRx(0.3, 0.2, 0.1), Point3(20, 10, 5), 1.0);

  Point3 b1(0, 0, 1), b2(10, 0, 2), b3(20, -10, 30);

  Point3Pair ab1(make_pair(expected_aSb.transformFrom(b1), b1));
  Point3Pair ab2(make_pair(expected_aSb.transformFrom(b2), b2));
  Point3Pair ab3(make_pair(expected_aSb.transformFrom(b3), b3));

  vector<Point3Pair> correspondences{ab1, ab2, ab3};

  Similarity3 actual_aSb = Similarity3::Align(correspondences);
  EXPECT(assert_equal(expected_aSb, actual_aSb));
}

//******************************************************************************
// Align with Pose3 Pairs
TEST(Similarity3, AlignPose3) {
  Similarity3 expected_wSe(Rot3::Ry(180 * degree), Point3(2, 3, 5), 2.0);

  // Create source poses (two objects o1 and o2 living in the egovehicle "e" frame)
  // Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
  Pose3 eTo1(Rot3(), Point3(0, 0, 0));
  Pose3 eTo2(Rot3(-1, 0, 0, 0, -1, 0, 0, 0, 1), Point3(4, 0, 0));

  // Create destination poses (same two objects, but instead living in the world/city "w" frame)
  Pose3 wTo1(Rot3(-1, 0, 0, 0, 1, 0, 0, 0, -1), Point3(4, 6, 10));
  Pose3 wTo2(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(-4, 6, 10));

  Pose3Pair wTe1(make_pair(wTo1, eTo1));
  Pose3Pair wTe2(make_pair(wTo2, eTo2));

  vector<Pose3Pair> correspondences{wTe1, wTe2};

  // Cayley transform cannot accommodate 180 degree rotations,
  // hence we only test for Expmap
#ifdef GTSAM_ROT3_EXPMAP
  // Recover the transformation wSe (i.e. world_S_egovehicle)
  Similarity3 actual_wSe = Similarity3::Align(correspondences);
  EXPECT(assert_equal(expected_wSe, actual_wSe));
#endif
}

//******************************************************************************
// Test very simple prior optimization example
TEST(Similarity3, Optimization) {
  // Create a PriorFactor with a Sim3 prior
  Similarity3 prior = Similarity3(Rot3::Ypr(0.1, 0.2, 0.3), Point3(1, 2, 3), 4);
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 1);
  Symbol key('x', 1);

  // Create graph
  NonlinearFactorGraph graph;
  graph.addPrior(key, prior, model);

  // Create initial estimate with Identity transform
  Values initial;
  initial.insert<Similarity3>(key, Similarity3());

  // Optimize
  Values result;
  LevenbergMarquardtParams params;
  params.setVerbosityLM("TRYCONFIG");
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  // After optimization, result should be prior
  EXPECT(assert_equal(prior, result.at<Similarity3>(key), 1e-4));
}

//******************************************************************************
// Test optimization with both Prior and BetweenFactors
TEST(Similarity3, Optimization2) {
  Similarity3 prior = Similarity3();
  Similarity3 m1 = Similarity3(Rot3::Ypr(M_PI / 4.0, 0, 0), Point3(2.0, 0, 0),
      1.0);
  Similarity3 m2 = Similarity3(Rot3::Ypr(M_PI / 2.0, 0, 0),
      Point3(sqrt(8) * 0.9, 0, 0), 1.0);
  Similarity3 m3 = Similarity3(Rot3::Ypr(3 * M_PI / 4.0, 0, 0),
      Point3(sqrt(32) * 0.8, 0, 0), 1.0);
  Similarity3 m4 = Similarity3(Rot3::Ypr(M_PI / 2.0, 0, 0),
      Point3(6 * 0.7, 0, 0), 1.0);
  Similarity3 loop = Similarity3(1.42);

  //prior.print("Goal Transform");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7,
      0.01);
  SharedDiagonal betweenNoise = noiseModel::Diagonal::Sigmas(
      (Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10).finished());
  SharedDiagonal betweenNoise2 = noiseModel::Diagonal::Sigmas(
      (Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0).finished());
  BetweenFactor<Similarity3> b1(X(1), X(2), m1, betweenNoise);
  BetweenFactor<Similarity3> b2(X(2), X(3), m2, betweenNoise);
  BetweenFactor<Similarity3> b3(X(3), X(4), m3, betweenNoise);
  BetweenFactor<Similarity3> b4(X(4), X(5), m4, betweenNoise);
  BetweenFactor<Similarity3> lc(X(5), X(1), loop, betweenNoise2);

  // Create graph
  NonlinearFactorGraph graph;
  graph.addPrior(X(1), prior, model); // Prior !
  graph.push_back(b1);
  graph.push_back(b2);
  graph.push_back(b3);
  graph.push_back(b4);
  graph.push_back(lc);

  //graph.print("Full Graph\n");
  Values initial;
  initial.insert<Similarity3>(X(1), Similarity3());
  initial.insert<Similarity3>(X(2),
      Similarity3(Rot3::Ypr(M_PI / 2.0, 0, 0), Point3(1, 0, 0), 1.1));
  initial.insert<Similarity3>(X(3),
      Similarity3(Rot3::Ypr(2.0 * M_PI / 2.0, 0, 0), Point3(0.9, 1.1, 0), 1.2));
  initial.insert<Similarity3>(X(4),
      Similarity3(Rot3::Ypr(3.0 * M_PI / 2.0, 0, 0), Point3(0, 1, 0), 1.3));
  initial.insert<Similarity3>(X(5),
      Similarity3(Rot3::Ypr(4.0 * M_PI / 2.0, 0, 0), Point3(0, 0, 0), 1.0));

  //initial.print("Initial Estimate\n");

  Values result;
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  //result.print("Optimized Estimate\n");
  Similarity3 p1, p2, p3, p4, p5;
  p1 = result.at<Similarity3>(X(1));
  p2 = result.at<Similarity3>(X(2));
  p3 = result.at<Similarity3>(X(3));
  p4 = result.at<Similarity3>(X(4));
  p5 = result.at<Similarity3>(X(5));

  //p1.print("Similarity1");
  //p2.print("Similarity2");
  //p3.print("Similarity3");
  //p4.print("Similarity4");
  //p5.print("Similarity5");

  Similarity3 expected(0.7);
  EXPECT(assert_equal(expected, result.at<Similarity3>(X(5)), 0.4));
}

//******************************************************************************
// Align points (p,q) assuming that p = T*q + noise
TEST(Similarity3, AlignScaledPointClouds) {
// Create ground truth
  Point3 q1(0, 0, 0), q2(1, 0, 0), q3(0, 1, 0);

  // Create transformed cloud (noiseless)
//  Point3 p1 = T4 * q1, p2 = T4 * q2, p3 = T4 * q3;

  // Create an unknown expression
  Expression<Similarity3> unknownT(0); // use key 0

  // Create constant expressions for the ground truth points
  Expression<Point3> q1_(q1), q2_(q2), q3_(q3);

  // Create prediction expressions
  Expression<Point3> predict1(unknownT, &Similarity3::transformFrom, q1_);
  Expression<Point3> predict2(unknownT, &Similarity3::transformFrom, q2_);
  Expression<Point3> predict3(unknownT, &Similarity3::transformFrom, q3_);

//// Create Expression factor graph
//  ExpressionFactorGraph graph;
//  graph.addExpressionFactor(predict1, p1, R); // |T*q1 - p1|
//  graph.addExpressionFactor(predict2, p2, R); // |T*q2 - p2|
//  graph.addExpressionFactor(predict3, p3, R); // |T*q3 - p3|
}

//******************************************************************************
TEST(Similarity3 , Invariants) {
  Similarity3 id;

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
TEST(Similarity3 , LieGroupDerivatives) {
  Similarity3 id;

  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2, id);
  CHECK_LIE_GROUP_DERIVATIVES(T2, T3);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

