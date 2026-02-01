/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testIncrementalFixedLagSmoother.cpp
 * @brief   Unit tests for the Incremental Fixed-Lag Smoother
 * @author  Stephen Williams (swilliams8@gatech.edu)
 * @date    May 23, 2012
 */

#include <gtsam/base/debug.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>  // For writeG2o
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <string>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using BetweenPoint2 = BetweenFactor<Point2>;

/* ************************************************************************* */
bool check_smoother(const NonlinearFactorGraph& fullgraph,
                    const Values& fullinit,
                    const IncrementalFixedLagSmoother& smoother,
                    const Key& key) {
  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit);
  VectorValues delta = linearized.optimize();
  Values fullfinal = fullinit.retract(delta);

  Point2 expected = fullfinal.at<Point2>(key);
  Point2 actual = smoother.calculateEstimate<Point2>(key);

  return assert_equal(expected, actual);
}

/* ************************************************************************* */
void PrintSymbolicTreeHelper(const ISAM2Clique::shared_ptr& clique,
                             const std::string indent = "") {
  // Print the current clique
  std::cout << indent << "P( ";
  for (Key key : clique->conditional()->frontals()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  if (clique->conditional()->nrParents() > 0) std::cout << "| ";
  for (Key key : clique->conditional()->parents()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  std::cout << ")" << std::endl;

  // Recursively print all of the children
  for (const ISAM2Clique::shared_ptr& child : clique->children) {
    PrintSymbolicTreeHelper(child, indent + " ");
  }
}

/* ************************************************************************* */
void PrintSymbolicTree(const ISAM2& isam, const std::string& label) {
  std::cout << label << std::endl;
  if (!isam.roots().empty()) {
    for (const ISAM2::sharedClique& root : isam.roots()) {
      PrintSymbolicTreeHelper(root);
    }
  } else
    std::cout << "{Empty Tree}" << std::endl;
}

/* ************************************************************************* */
TEST(IncrementalFixedLagSmoother, Example) {
  // Test the IncrementalFixedLagSmoother in a pure linear environment. Thus,
  // full optimization and the IncrementalFixedLagSmoother should be identical
  // (even with the linearized approximations at the end of the smoothing lag)

  SETDEBUG("IncrementalFixedLagSmoother update", true);

  // Set up parameters
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
  SharedDiagonal loopNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

  // Create a Fixed-Lag Smoother
  typedef IncrementalFixedLagSmoother::KeyTimestampMap Timestamps;
  IncrementalFixedLagSmoother smoother(12.0, ISAM2Params());

  // Create containers to keep the full graph
  Values fullinit;
  NonlinearFactorGraph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update the HMF
  {
    Key key0 = X(0);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.addPrior(key0, Point2(0.0, 0.0), odoNoise);
    newValues.insert(key0, Point2(0.01, 0.01));
    newTimestamps[key0] = 0.0;

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key0));

    ++i;
  }

  // Add odometry from time 0 to time 5
  while (i <= 5) {
    Key key1 = X(i - 1);
    Key key2 = X(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newValues.insert(key2, Point2(double(i) + 0.1, -0.1));
    newTimestamps[key2] = double(i);

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

    ++i;
  }

  // Add odometry from time 5 to 6 to the HMF and a loop closure at time 5 to
  // the TSM
  {
    // Add the odometry factor to the HMF
    Key key1 = X(i - 1);
    Key key2 = X(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newFactors.emplace_shared<BetweenPoint2>(X(2), X(5), Point2(3.5, 0.0),
                                             loopNoise);
    newValues.insert(key2, Point2(double(i) + 0.1, -0.1));
    newTimestamps[key2] = double(i);

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

    ++i;
  }

  // Add odometry from time 6 to time 15
  while (i <= 15) {
    Key key1 = X(i - 1);
    Key key2 = X(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    // Add the odometry factor twice to ensure the removeFactor test below
    // works, where we need to keep the connectivity of the graph.
    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newValues.insert(key2, Point2(double(i) + 0.1, -0.1));
    newTimestamps[key2] = double(i);

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

    ++i;
  }

  // add/remove an extra factor
  {
    Key key1 = X(i - 1);
    Key key2 = X(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    // add 2 odometry factors
    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newFactors.emplace_shared<BetweenPoint2>(key1, key2, Point2(1.0, 0.0),
                                             odoNoise);
    newValues.insert(key2, Point2(double(i) + 0.1, -0.1));
    newTimestamps[key2] = double(i);
    ++i;

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

    // now remove one of the two and try again
    // empty values and new factors for fake update in which we only remove
    // factors
    NonlinearFactorGraph emptyNewFactors;
    Values emptyNewValues;
    Timestamps emptyNewTimestamps;

    size_t factorIndex =
        25;  // any index that does not break connectivity of the graph
    FactorIndices factorToRemove;
    factorToRemove.push_back(factorIndex);

    const NonlinearFactorGraph smootherFactorsBeforeRemove =
        smoother.getFactors();

    std::cout << "fullgraph.size() = " << fullgraph.size() << std::endl;
    std::cout << "smootherFactorsBeforeRemove.size() = "
              << smootherFactorsBeforeRemove.size() << std::endl;

    // remove factor
    smoother.update(emptyNewFactors, emptyNewValues, emptyNewTimestamps,
                    factorToRemove);

    // Note: the following test (checking that the number of factor is reduced
    // by 1) fails  since we are not reusing slots, hence also when removing a
    // factor we do not change the size of the factor graph size_t
    // nrFactorsAfterRemoval = smoother.getFactors().size();
    // DOUBLES_EQUAL(nrFactorsBeforeRemoval-1, nrFactorsAfterRemoval, 1e-5);

    // check that the factors in the smoother are right
    NonlinearFactorGraph actual = smoother.getFactors();
    for (size_t i = 0; i < smootherFactorsBeforeRemove.size(); i++) {
      // check that the factors that were not removed are there
      if (smootherFactorsBeforeRemove[i] && i != factorIndex) {
        EXPECT(smootherFactorsBeforeRemove[i]->equals(*actual[i]));
      } else {  // while the factors that were not there or were removed are no
                // longer there
        EXPECT(!actual[i]);
      }
    }
  }

  {
    SETDEBUG("BayesTreeMarginalizationHelper", true);
    PrintSymbolicTree(smoother.getISAM2(),
                      "Bayes Tree Before marginalization test:");

    // Do pressure test on marginalization. Enlarge max_i to enhance the test.
    const int max_i = 500;
    while (i <= max_i) {
      Key key_0 = X(i);
      Key key_1 = X(i - 1);
      Key key_2 = X(i - 2);
      Key key_3 = X(i - 3);
      Key key_4 = X(i - 4);
      Key key_5 = X(i - 5);
      Key key_6 = X(i - 6);
      Key key_7 = X(i - 7);
      Key key_8 = X(i - 8);
      Key key_9 = X(i - 9);
      Key key_10 = X(i - 10);

      NonlinearFactorGraph newFactors;
      Values newValues;
      Timestamps newTimestamps;

      // To make a complex graph
      const Point2 z(1.0, 0.0);
      newFactors.emplace_shared<BetweenPoint2>(key_1, key_0, z, odoNoise);
      if (i % 2 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_2, key_1, z, odoNoise);
      if (i % 3 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_3, key_2, z, odoNoise);
      if (i % 4 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_4, key_3, z, odoNoise);
      if (i % 5 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_5, key_4, z, odoNoise);
      if (i % 6 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_6, key_5, z, odoNoise);
      if (i % 7 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_7, key_6, z, odoNoise);
      if (i % 8 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_8, key_7, z, odoNoise);
      if (i % 9 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_9, key_8, z, odoNoise);
      if (i % 10 == 0)
        newFactors.emplace_shared<BetweenPoint2>(key_10, key_9, z, odoNoise);

      newValues.insert(key_0, Point2(double(i) + 0.1, -0.1));
      newTimestamps[key_0] = double(i);

      fullgraph.push_back(newFactors);
      fullinit.insert(newValues);

      // Update the smoother
      smoother.update(newFactors, newValues, newTimestamps);

      // Check
      CHECK(check_smoother(fullgraph, fullinit, smoother, key_0));
      PrintSymbolicTree(
          smoother.getISAM2(),
          "Bayes Tree marginalization test: i = " + std::to_string(i));

      ++i;
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
