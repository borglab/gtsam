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

#include <gtsam/slam/InitializePose.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>

#include <cmath>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(InitializePose3, computePoses2D) {
  const string g2oFile = findExampleDataFile("noisyToyGraph.txt");
  NonlinearFactorGraph::shared_ptr inputGraph;
  Values::shared_ptr posesInFile;
  bool is3D = false;
  boost::tie(inputGraph, posesInFile) = readG2o(g2oFile, is3D);

  auto priorModel = noiseModel::Unit::Create(3);
  inputGraph->addPrior(0, posesInFile->at<Pose2>(0), priorModel);

  auto poseGraph = initialize::buildPoseGraph<Pose2>(*inputGraph);

  auto I = genericValue(Rot3());
  Values orientations;
  for (size_t i : {0, 1, 2, 3})
    orientations.insert(i, posesInFile->at<Pose2>(i).rotation());
  const Values poses = initialize::computePoses<Pose2>(orientations, &poseGraph);

  // posesInFile is seriously noisy, so we check error of recovered poses
  EXPECT_DOUBLES_EQUAL(0.0810283, inputGraph->error(poses), 1e-6);
}

/* ************************************************************************* */
TEST(InitializePose3, computePoses3D) {
  const string g2oFile = findExampleDataFile("Klaus3");
  NonlinearFactorGraph::shared_ptr inputGraph;
  Values::shared_ptr posesInFile;
  bool is3D = true;
  boost::tie(inputGraph, posesInFile) = readG2o(g2oFile, is3D);

  auto priorModel = noiseModel::Unit::Create(6);
  inputGraph->addPrior(0, posesInFile->at<Pose3>(0), priorModel);

  auto poseGraph = initialize::buildPoseGraph<Pose3>(*inputGraph);

  auto I = genericValue(Rot3());
  Values orientations;
  for (size_t i : {0, 1, 2})
    orientations.insert(i, posesInFile->at<Pose3>(i).rotation());
  Values poses = initialize::computePoses<Pose3>(orientations, &poseGraph);
  EXPECT(assert_equal(*posesInFile, poses, 0.1));  // TODO(frank): very loose !!
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
