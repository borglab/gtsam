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
#include <gtsam/slam/PriorFactor.h>
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
static Rot3   R0 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,0.0 ) );
static Point3 p1 = Point3(1,2,0);
static Rot3   R1 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,1.570796 ) );
static Point3 p2 = Point3(0,2,0);
static Rot3   R2 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,3.141593 ) );
static Point3 p3 = Point3(-1,1,0);
static Rot3   R3 = Rot3::Expmap( ( Vector(3) << 0.0,0.0,4.712389 ) );

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
  g.add(PriorFactor<Pose3>(x0, pose0, model));
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
  Values initial = InitializePose3::initializeOrientations(simple::graph());

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal(simple::R0, initial.at<Rot3>(x0), 1e-6));
  EXPECT(assert_equal(simple::R1, initial.at<Rot3>(x1), 1e-6));
  EXPECT(assert_equal(simple::R2, initial.at<Rot3>(x2), 1e-6));
  EXPECT(assert_equal(simple::R3, initial.at<Rot3>(x3), 1e-6));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

