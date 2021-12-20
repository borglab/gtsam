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

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/base/debug.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;


/* ************************************************************************* */
bool check_smoother(const NonlinearFactorGraph& fullgraph, const Values& fullinit, const BatchFixedLagSmoother& smoother, const Key& key) {

  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit);
  VectorValues delta = linearized.optimize();
  Values fullfinal = fullinit.retract(delta);

  Point2 expected = fullfinal.at<Point2>(key);
  Point2 actual = smoother.calculateEstimate<Point2>(key);

  return assert_equal(expected, actual);
}

/* ************************************************************************* */
TEST( BatchFixedLagSmoother, Example )
{
  // Test the BatchFixedLagSmoother in a pure linear environment. Thus, full optimization and
  // the BatchFixedLagSmoother should be identical (even with the linearized approximations at
  // the end of the smoothing lag)

  //  SETDEBUG("BatchFixedLagSmoother update", true);
  //  SETDEBUG("BatchFixedLagSmoother reorder", true);
  //  SETDEBUG("BatchFixedLagSmoother optimize", true);
  //  SETDEBUG("BatchFixedLagSmoother marginalize", true);
  //  SETDEBUG("BatchFixedLagSmoother calculateMarginalFactors", true);

  // Set up parameters
  SharedDiagonal odometerNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
  SharedDiagonal loopNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

  // Create a Fixed-Lag Smoother
  typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;
  BatchFixedLagSmoother smoother(7.0, LevenbergMarquardtParams());

  // Create containers to keep the full graph
  Values fullinit;
  NonlinearFactorGraph fullgraph;



  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update the HMF
  {
    Key key0(0);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.addPrior(key0, Point2(0.0, 0.0), odometerNoise);
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
  while(i <= 5) {
    Key key1(i-1);
    Key key2(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0), odometerNoise));
    newValues.insert(key2, Point2(double(i)+0.1, -0.1));
    newTimestamps[key2] = double(i);

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

    ++i;
  }

  // Add odometry from time 5 to 6 to the HMF and a loop closure at time 5 to the TSM
  {
    // Add the odometry factor to the HMF
    Key key1(i-1);
    Key key2(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0), odometerNoise));
    newFactors.push_back(BetweenFactor<Point2>(Key(2), Key(5), Point2(3.5, 0.0), loopNoise));
    newValues.insert(key2, Point2(double(i)+0.1, -0.1));
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
  while(i <= 15) {
    Key key1(i-1);
    Key key2(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0), odometerNoise));
    newValues.insert(key2, Point2(double(i)+0.1, -0.1));
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
    Key key1 = Key(i-1);
    Key key2 = Key(i);

    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    // add 2 odometry factors
    newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0), odometerNoise));
    newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0), odometerNoise));
    newValues.insert(key2, Point2(double(i)+0.1, -0.1));
    newTimestamps[key2] = double(i);

    fullgraph.push_back(newFactors);
    fullinit.insert(newValues);

    // Update the smoother
    smoother.update(newFactors, newValues, newTimestamps);

    // Check
    CHECK(check_smoother(fullgraph, fullinit, smoother, key2));

//    NonlinearFactorGraph smootherGraph = smoother.getFactors();
//    for(size_t i=0; i<smootherGraph.size(); i++){
//      if(smootherGraph[i]){
//      std::cout << "i:" << i << std::endl;
//      smootherGraph[i]->print();
//      }
//    }

    // now remove one of the two and try again
    // empty values and new factors for fake update in which we only remove factors
    NonlinearFactorGraph emptyNewFactors;
    Values emptyNewValues;
    Timestamps emptyNewTimestamps;

    size_t factorIndex = 6; // any index that does not break connectivity of the graph
    FactorIndices factorToRemove;
    factorToRemove.push_back(factorIndex);

    const NonlinearFactorGraph smootherFactorsBeforeRemove = smoother.getFactors();

    // remove factor
    smoother.update(emptyNewFactors, emptyNewValues, emptyNewTimestamps,factorToRemove);

    // check that the factors in the smoother are right
    NonlinearFactorGraph actual = smoother.getFactors();
    for(size_t i=0; i< smootherFactorsBeforeRemove.size(); i++){
      // check that the factors that were not removed are there
      if(smootherFactorsBeforeRemove[i] && i != factorIndex){
        EXPECT(smootherFactorsBeforeRemove[i]->equals(*actual[i]));
      }
      else{ // while the factors that were not there or were removed are no longer there
        EXPECT(!actual[i]);
      }
    }
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
