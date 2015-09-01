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
 */

#include <gtsam_unstable/geometry/Similarity3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using symbol_shorthand::X;

GTSAM_CONCEPT_TESTABLE_INST(Similarity3)

static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::Rodrigues(0.3,0,0);
static Similarity3 T(R,Point3(3.5,-8.2,4.2),1);
static Similarity3 T2(Rot3::Rodrigues(0.3,0.2,0.1),Point3(3.5,-8.2,4.2),1);
static Similarity3 T3(Rot3::Rodrigues(-90, 0, 0), Point3(1, 2, 3), 1);

//******************************************************************************
TEST(Similarity3, Constructors) {
  Similarity3 test;
}

//******************************************************************************
TEST(Similarity3, Getters) {
  Similarity3 test;
  EXPECT(assert_equal(Rot3(), test.rotation()));
  EXPECT(assert_equal(Point3(), test.translation()));
  EXPECT_DOUBLES_EQUAL(1.0, test.scale(), 1e-9);
}

//******************************************************************************
TEST(Similarity3, Getters2) {
  Similarity3 test(Rot3::ypr(1, 2, 3), Point3(4, 5, 6), 7);
  EXPECT(assert_equal(Rot3::ypr(1, 2, 3), test.rotation()));
  EXPECT(assert_equal(Point3(4, 5, 6), test.translation()));
  EXPECT_DOUBLES_EQUAL(7.0, test.scale(), 1e-9);
}

TEST(Similarity3, AdjointMap) {
  Similarity3 test(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Matrix7 result;
  result <<    -1.5739,   -2.4512,   -6.3651,  -50.7671,  -11.2503,   16.8859,  -28.0000,
      6.3167,   -2.9884,   -0.4111,    0.8502,    8.6373,  -49.7260,  -35.0000,
     -2.5734,   -5.8362,    2.8839,   33.1363,    0.3024,   30.1811,  -42.0000,
           0,         0,         0,   -0.2248,   -0.3502,   -0.9093,         0,
           0,         0,         0,    0.9024,   -0.4269,   -0.0587,         0,
           0,         0,         0,   -0.3676,   -0.8337,    0.4120,         0,
           0,         0,         0,         0,         0,         0,    1.0000;
  EXPECT(assert_equal(result, test.AdjointMap(), 1e-3));
}

TEST(Similarity3, inverse) {
  Similarity3 test(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Matrix3 Re;
  Re <<    -0.2248,    0.9024,   -0.3676,
   -0.3502,   -0.4269,   -0.8337,
   -0.9093,   -0.0587,    0.4120;
  Vector3 te(-9.8472, 59.7640, 10.2125);
  Similarity3 expected(Re, te, 1.0/7.0);
  EXPECT(assert_equal(expected, test.inverse(), 1e-3));
}

TEST(Similarity3, multiplication) {
  Similarity3 test1(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Similarity3 test2(Rot3::ypr(1,2,3).inverse(), Point3(8,9,10), 11);
  Matrix3 re;
  re << 0.0688,    0.9863,   -0.1496,
     -0.5665,   -0.0848,   -0.8197,
     -0.8211,    0.1412,    0.5530;
  Vector3 te(-13.6797, 3.2441, -5.7794);
  Similarity3 expected(re, te, 77);
  EXPECT(assert_equal(expected, test1*test2, 1e-2));
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

//  Similarity3 other = Similarity3(Rot3::ypr(0.01, 0.02, 0.03), Point3(0.4, 0.5, 0.6), 1);
  Similarity3 other = Similarity3(Rot3::ypr(0.1, 0.2, 0.3),Point3(4,5,6),1);

  Vector vlocal = sim.localCoordinates(other);

  EXPECT(assert_equal(sim.retract(vlocal), other, 1e-2));

  Similarity3 other2 = Similarity3(Rot3::ypr(0.3, 0, 0),Point3(4,5,6),1);
  Rot3 R = Rot3::Rodrigues(0.3,0,0);

  Vector vlocal2 = sim.localCoordinates(other2);

  EXPECT(assert_equal(sim.retract(vlocal2), other2, 1e-2));

  // TODO add unit tests for retract and localCoordinates
}

/* ************************************************************************* */
TEST( Similarity3, retract_first_order)
{
  Similarity3 id;
  Vector v = zero(7);
  v(0) = 0.3;
  EXPECT(assert_equal(Similarity3(R, Point3(), 1), id.retract(v),1e-2));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  EXPECT(assert_equal(Similarity3(R, P, 1),id.retract(v),1e-2));
}

/* ************************************************************************* */
TEST(Similarity3, localCoordinates_first_order)
{
  Vector d12 = repeat(7,0.1);
  d12(6) = 1.0;
  Similarity3 t1 = T, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}

/* ************************************************************************* */
TEST(Similarity3, manifold_first_order)
{
  Similarity3 t1 = T;
  Similarity3 t2 = T3;
  Similarity3 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
}

TEST(Similarity3, Optimization) {
  Similarity3 prior = Similarity3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 2, 3), 4);
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 1);
  Symbol key('x',1);
  PriorFactor<Similarity3> factor(key, prior, model);

  NonlinearFactorGraph graph;
  graph.push_back(factor);

  Values initial;
  initial.insert<Similarity3>(key, Similarity3());

  Values result;
  LevenbergMarquardtParams params;
  params.setVerbosityLM("TRYCONFIG");
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  EXPECT(assert_equal(prior, result.at<Similarity3>(key), 1e-4));
}

TEST(Similarity3, Optimization2) {
  Similarity3 prior = Similarity3();
  Similarity3 m1 = Similarity3(Rot3::ypr(M_PI/4.0, 0, 0), Point3(2.0, 0, 0), 1.0);
  Similarity3 m2 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(sqrt(8)*0.9, 0, 0), 1.0);
  Similarity3 m3 = Similarity3(Rot3::ypr(3*M_PI/4.0, 0, 0), Point3(sqrt(32)*0.8, 0, 0), 1.0);
  Similarity3 m4 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(6*0.7, 0, 0), 1.0);
  Similarity3 loop = Similarity3(1.42);

  //prior.print("Goal Transform");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 0.01);
  SharedDiagonal betweenNoise = noiseModel::Diagonal::Sigmas(
        (Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10).finished());
  SharedDiagonal betweenNoise2 = noiseModel::Diagonal::Sigmas(
          (Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0).finished());
  PriorFactor<Similarity3> factor(X(1), prior, model);
  BetweenFactor<Similarity3> b1(X(1), X(2), m1, betweenNoise);
  BetweenFactor<Similarity3> b2(X(2), X(3), m2, betweenNoise);
  BetweenFactor<Similarity3> b3(X(3), X(4), m3, betweenNoise);
  BetweenFactor<Similarity3> b4(X(4), X(5), m4, betweenNoise);
  BetweenFactor<Similarity3> lc(X(5), X(1), loop, betweenNoise2);



  NonlinearFactorGraph graph;
  graph.push_back(factor);
  graph.push_back(b1);
  graph.push_back(b2);
  graph.push_back(b3);
  graph.push_back(b4);
  graph.push_back(lc);

  //graph.print("Full Graph\n");

  Values initial;
  initial.insert<Similarity3>(X(1), Similarity3());
  initial.insert<Similarity3>(X(2), Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(1, 0, 0), 1.1));
  initial.insert<Similarity3>(X(3), Similarity3(Rot3::ypr(2.0*M_PI/2.0, 0, 0), Point3(0.9, 1.1, 0), 1.2));
  initial.insert<Similarity3>(X(4), Similarity3(Rot3::ypr(3.0*M_PI/2.0, 0, 0), Point3(0, 1, 0), 1.3));
  initial.insert<Similarity3>(X(5), Similarity3(Rot3::ypr(4.0*M_PI/2.0, 0, 0), Point3(0, 0, 0), 1.0));

  //initial.print("Initial Estimate\n");

  Values result;
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  //result.print("Optimized Estimate\n");
  Pose3 p1, p2, p3, p4, p5;
  p1 = Pose3(result.at<Similarity3>(X(1)));
  p2 = Pose3(result.at<Similarity3>(X(2)));
  p3 = Pose3(result.at<Similarity3>(X(3)));
  p4 = Pose3(result.at<Similarity3>(X(4)));
  p5 = Pose3(result.at<Similarity3>(X(5)));

  //p1.print("Pose1");
  //p2.print("Pose2");
  //p3.print("Pose3");
  //p4.print("Pose4");
  //p5.print("Pose5");

  Similarity3 expected(0.7);
  EXPECT(assert_equal(expected, result.at<Similarity3>(X(5)), 0.4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

