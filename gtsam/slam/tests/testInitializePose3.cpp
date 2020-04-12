/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testInitializePose3.cpp
 *  @brief Unit tests for 3D SLAM initialization, using rotation relaxation
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   August, 2014
 */

#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <CppUnitLite/TestHarness.h>

#include <cmath>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

static Symbol x0('x', 0), x1('x', 1), x2('x', 2), x3('x', 3);
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(6, 0.1));

namespace simple {
// We consider a small graph:
//                            symbolic FG
//               x2               0  1
//             / | \              1  2
//            /  |  \             2  3
//          x3   |   x1           2  0
//           \   |   /            0  3
//            \  |  /
//               x0
//
static Point3 p0 = Point3(0,0,0);
static Rot3   R0 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,0.0 ).finished() );
static Point3 p1 = Point3(1,2,0);
static Rot3   R1 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,1.570796 ).finished() );
static Point3 p2 = Point3(0,2,0);
static Rot3   R2 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,3.141593 ).finished() );
static Point3 p3 = Point3(-1,1,0);
static Rot3   R3 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,4.712389 ).finished() );

static Pose3 pose0 = Pose3(R0,p0);
static Pose3 pose1 = Pose3(R1,p1);
static Pose3 pose2 = Pose3(R2,p2);
static Pose3 pose3 = Pose3(R3,p3);

NonlinearFactorGraph graph() {
  NonlinearFactorGraph g;
  g.add(BetweenFactor<Pose3>(x0, x1, pose0.between(pose1), model));
  g.add(BetweenFactor<Pose3>(x1, x2, pose1.between(pose2), model));
  g.add(BetweenFactor<Pose3>(x2, x3, pose2.between(pose3), model));
  g.add(BetweenFactor<Pose3>(x2, x0, pose2.between(pose0), model));
  g.add(BetweenFactor<Pose3>(x0, x3, pose0.between(pose3), model));
  g.addPrior(x0, pose0, model);
  return g;
}

NonlinearFactorGraph graph2() {
  NonlinearFactorGraph g;
  g.add(BetweenFactor<Pose3>(x0, x1, pose0.between(pose1), noiseModel::Isotropic::Precision(6, 1.0)));
  g.add(BetweenFactor<Pose3>(x1, x2, pose1.between(pose2), noiseModel::Isotropic::Precision(6, 1.0)));
  g.add(BetweenFactor<Pose3>(x2, x3, pose2.between(pose3), noiseModel::Isotropic::Precision(6, 1.0)));
  g.add(BetweenFactor<Pose3>(x2, x0, Pose3(Rot3::Ypr(0.1,0,0.1), Point3()), noiseModel::Isotropic::Precision(6, 0.0))); // random pose, but zero information
  g.add(BetweenFactor<Pose3>(x0, x3, Pose3(Rot3::Ypr(0.5,-0.2,0.2), Point3(10,20,30)), noiseModel::Isotropic::Precision(6, 0.0))); // random pose, but zero informatoin
  g.addPrior(x0, pose0, model);
  return g;
}
}

/* *************************************************************************** */
TEST( InitializePose3, buildPose3graph ) {
  NonlinearFactorGraph pose3graph = InitializePose3::buildPose3graph(simple::graph());
  // pose3graph.print("");
}

/* *************************************************************************** */
TEST( InitializePose3, orientations ) {
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(simple::graph());

  Values initial = InitializePose3::computeOrientationsChordal(pose3Graph);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal(simple::R0, initial.at<Rot3>(x0), 1e-6));
  EXPECT(assert_equal(simple::R1, initial.at<Rot3>(x1), 1e-6));
  EXPECT(assert_equal(simple::R2, initial.at<Rot3>(x2), 1e-6));
  EXPECT(assert_equal(simple::R3, initial.at<Rot3>(x3), 1e-6));
}

/* *************************************************************************** */
TEST( InitializePose3, orientationsPrecisions ) {
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(simple::graph2());

  Values initial = InitializePose3::computeOrientationsChordal(pose3Graph);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal(simple::R0, initial.at<Rot3>(x0), 1e-6));
  EXPECT(assert_equal(simple::R1, initial.at<Rot3>(x1), 1e-6));
  EXPECT(assert_equal(simple::R2, initial.at<Rot3>(x2), 1e-6));
  EXPECT(assert_equal(simple::R3, initial.at<Rot3>(x3), 1e-6));
}

/* *************************************************************************** */
TEST( InitializePose3, orientationsGradientSymbolicGraph ) {
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(simple::graph());

  KeyVectorMap adjEdgesMap;
  KeyRotMap factorId2RotMap;

  InitializePose3::createSymbolicGraph(adjEdgesMap, factorId2RotMap, pose3Graph);

  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x0)[0], 0, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x0)[1], 3, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x0)[2], 4, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x0)[3], 5, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x0).size(), 4, 1e-9);

  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x1)[0], 0, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x1)[1], 1, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x1).size(), 2, 1e-9);

  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x2)[0], 1, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x2)[1], 2, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x2)[2], 3, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x2).size(), 3, 1e-9);

  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x3)[0], 2, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x3)[1], 4, 1e-9);
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.at(x3).size(), 2, 1e-9);

  // This includes the anchor
  EXPECT_DOUBLES_EQUAL(adjEdgesMap.size(), 5, 1e-9);
}

/* *************************************************************************** */
TEST( InitializePose3, singleGradient ) {
  Rot3 R1 = Rot3();
  Matrix M = Matrix3::Zero();
  M(0,1) = -1; M(1,0) = 1; M(2,2) = 1;
  Rot3 R2 = Rot3(M);
  double a = 6.010534238540223;
  double b = 1.0;

  Vector actual = InitializePose3::gradientTron(R1, R2, a, b);
  Vector expected = Vector3::Zero();
  expected(2) = 1.962658662803917;

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( InitializePose3, iterationGradient ) {
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(simple::graph());

  // Wrong initial guess - initialization should fix the rotations
  Rot3 Rpert = Rot3::Expmap(Vector3(0.01, 0.01, 0.01));
  Values givenPoses;
  givenPoses.insert(x0,simple::pose0);
  givenPoses.insert(x1,(simple::pose0).compose( Pose3(Rpert,Point3(0,0,0)) ));
  givenPoses.insert(x2, (simple::pose0).compose( Pose3(Rpert.inverse(),Point3(0,0,0)) ));
  givenPoses.insert(x3, (simple::pose0).compose( Pose3(Rpert,Point3(0,0,0)) ));

  size_t maxIter = 1; // test gradient at the first iteration
  bool setRefFrame = false;
  Values orientations = InitializePose3::computeOrientationsGradient(pose3Graph, givenPoses, maxIter, setRefFrame);

  Matrix M0 = (Matrix(3,3) <<  0.999435813876064,  -0.033571481675497,   0.001004768630281,
      0.033572116359134,   0.999436104312325,  -0.000621610948719,
     -0.000983333645009,   0.000654992453817,   0.999999302019670).finished();
  Rot3 R0Expected = Rot3(M0);
  EXPECT(assert_equal(R0Expected, orientations.at<Rot3>(x0), 1e-5));

  Matrix M1 = (Matrix(3,3) <<  0.999905367545392,  -0.010866391403031,   0.008436675399114,
      0.010943459008004,   0.999898317528125,  -0.009143047050380,
     -0.008336465609239,   0.009234508232789,   0.999922610604863).finished();
  Rot3 R1Expected = Rot3(M1);
  EXPECT(assert_equal(R1Expected, orientations.at<Rot3>(x1), 1e-5));

  Matrix M2 = (Matrix(3,3) <<   0.998936644682875,   0.045376417678595,  -0.008158469732553,
      -0.045306446926148,   0.998936408933058,   0.008566024448664,
       0.008538487960253,  -0.008187284445083,   0.999930028850403).finished();
  Rot3 R2Expected = Rot3(M2);
  EXPECT(assert_equal(R2Expected, orientations.at<Rot3>(x2), 1e-5));

  Matrix M3 = (Matrix(3,3) <<   0.999898767273093,  -0.010834701971459,   0.009223038487275,
      0.010911315499947,   0.999906044037258,  -0.008297366559388,
     -0.009132272433995,   0.008397162077148,   0.999923041673329).finished();
  Rot3 R3Expected = Rot3(M3);
  EXPECT(assert_equal(R3Expected, orientations.at<Rot3>(x3), 1e-5));
}

/* *************************************************************************** */
TEST( InitializePose3, orientationsGradient ) {
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(simple::graph());

  // Wrong initial guess - initialization should fix the rotations
  Rot3 Rpert = Rot3::Expmap(Vector3(0.01, 0.01, 0.01));
  Values givenPoses;
  givenPoses.insert(x0,simple::pose0);
  givenPoses.insert(x1,(simple::pose0).compose( Pose3(Rpert,Point3(0,0,0)) ));
  givenPoses.insert(x2, (simple::pose0).compose( Pose3(Rpert.inverse(),Point3(0,0,0)) ));
  givenPoses.insert(x3, (simple::pose0).compose( Pose3(Rpert,Point3(0,0,0)) ));
  // do 10 gradient iterations
  bool setRefFrame = false;
  Values orientations = InitializePose3::computeOrientationsGradient(pose3Graph, givenPoses, 10, setRefFrame);

  //  const Key keyAnchor = symbol('Z', 9999999);
  //  givenPoses.insert(keyAnchor,simple::pose0);
  //  string g2oFile = "/home/aspn/Desktop/toyExample.g2o";
  //  writeG2o(pose3Graph, givenPoses, g2oFile);

  const string matlabResultsfile = findExampleDataFile("simpleGraph10gradIter");
  NonlinearFactorGraph::shared_ptr matlabGraph;
  Values::shared_ptr matlabValues;
  bool is3D = true;
  boost::tie(matlabGraph, matlabValues) = readG2o(matlabResultsfile, is3D);

  Rot3 R0Expected = matlabValues->at<Pose3>(1).rotation();
  EXPECT(assert_equal(R0Expected, orientations.at<Rot3>(x0), 1e-4));

  Rot3 R1Expected = matlabValues->at<Pose3>(2).rotation();
  EXPECT(assert_equal(R1Expected, orientations.at<Rot3>(x1), 1e-4));

  Rot3 R2Expected = matlabValues->at<Pose3>(3).rotation();
  EXPECT(assert_equal(R2Expected, orientations.at<Rot3>(x2), 1e-3));

  Rot3 R3Expected = matlabValues->at<Pose3>(4).rotation();
  EXPECT(assert_equal(R3Expected, orientations.at<Rot3>(x3), 1e-4));
}

/* *************************************************************************** */
TEST( InitializePose3, posesWithGivenGuess ) {
  Values givenPoses;
  givenPoses.insert(x0,simple::pose0);
  givenPoses.insert(x1,simple::pose1);
  givenPoses.insert(x2,simple::pose2);
  givenPoses.insert(x3,simple::pose3);

  Values initial = InitializePose3::initialize(simple::graph(), givenPoses);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal(givenPoses, initial, 1e-6));
}

/* ************************************************************************* */
TEST( InitializePose3, initializePoses )
{
  const string g2oFile = findExampleDataFile("pose3example-grid");
  NonlinearFactorGraph::shared_ptr inputGraph;
  Values::shared_ptr expectedValues;
  bool is3D = true;
  boost::tie(inputGraph, expectedValues) = readG2o(g2oFile, is3D);
  noiseModel::Unit::shared_ptr priorModel = noiseModel::Unit::Create(6);
  inputGraph->addPrior(0, Pose3(), priorModel);

  Values initial = InitializePose3::initialize(*inputGraph);
  EXPECT(assert_equal(*expectedValues,initial,0.1));  // TODO(frank): very loose !!
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

