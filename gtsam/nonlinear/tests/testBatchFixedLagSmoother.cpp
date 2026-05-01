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
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/debug.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <random>

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
TEST( BatchFixedLagSmoother, EnforceConsistency )
{
  // Verify that enforceConsistency_ actually preserves linearization points
  // for variables involved in marginal factors after marginalization.
  // Before the fix, linearValues_ was never populated, so this feature
  // was silently non-functional.

  SharedDiagonal noise = noiseModel::Isotropic::Sigma(2, 0.1);

  typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;

  // Create two smoothers: one with consistency enforcement, one without
  LevenbergMarquardtParams params;
  BatchFixedLagSmoother smootherOn(3.0, params, true);   // enforceConsistency = true
  BatchFixedLagSmoother smootherOff(3.0, params, false);  // enforceConsistency = false

  // Feed both smoothers the same data: a chain of between factors with
  // deliberately poor initial values to make relinearization matter.
  for (size_t i = 0; i <= 7; ++i) {
    NonlinearFactorGraph newFactors;
    Values newValues;
    Timestamps newTimestamps;

    Key key_i(i);
    if (i == 0) {
      newFactors.addPrior(key_i, Point2(0.0, 0.0), noise);
    } else {
      Key key_prev(i - 1);
      newFactors.push_back(BetweenFactor<Point2>(key_prev, key_i, Point2(1.0, 0.0), noise));
    }

    // Use a deliberately poor initial estimate to create nonlinearity
    newValues.insert(key_i, Point2(double(i) + 0.5, 0.5));
    newTimestamps[key_i] = double(i);

    smootherOn.update(newFactors, newValues, newTimestamps);
    smootherOff.update(newFactors, newValues, newTimestamps);
  }

  // After enough steps, marginalization has occurred (lag=3, at step 7 keys
  // 0..3 are marginalized). The smoothers should still produce valid estimates
  // but may differ because consistency enforcement constrains the optimization.
  // The key test: the enforceConsistency=true smoother should not crash and
  // should produce a reasonable estimate.
  Key lastKey(7);
  Point2 estimateOn = smootherOn.calculateEstimate<Point2>(lastKey);
  Point2 estimateOff = smootherOff.calculateEstimate<Point2>(lastKey);

  // Both should be close to the ground truth (7.0, 0.0) -- the chain of
  // unit between-factors from the origin.
  Point2 expected(7.0, 0.0);
  EXPECT(assert_equal(expected, estimateOn, 0.5));
  EXPECT(assert_equal(expected, estimateOff, 0.5));
}

/* ************************************************************************* */
TEST( BatchFixedLagSmoother, NEES )
{
  // Monte Carlo NEES evaluation comparing enforceConsistency on vs off.
  // Uses Pose2 (x, y, theta) so the problem is genuinely nonlinear --
  // the rotation makes Jacobians depend on the linearization point,
  // which is exactly where FEJ (First Estimates Jacobian) matters.

  const double transSigma = 0.5;
  const double rotSigma = 0.3;  // radians (~17 degrees)
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << rotSigma, transSigma, transSigma).finished());

  const size_t numTrials = 100;
  const size_t numSteps = 30;
  const double lag = 3.0;  // short lag forces more marginalization
  const size_t stateDim = 3;  // Pose2: (theta, x, y)

  // Ground truth: a curved trajectory with significant turns
  vector<Pose2> groundTruth(numSteps + 1);
  groundTruth[0] = Pose2(0, 0, 0);
  const Pose2 odomGT(1.0, 0.0, 0.4);  // 1m forward, 0.4 rad turn (~23 deg)
  for (size_t i = 1; i <= numSteps; ++i) {
    groundTruth[i] = groundTruth[i-1] * odomGT;
  }

  double neesSum_on = 0.0, neesSum_off = 0.0;
  size_t neesCount = 0;

  mt19937 rng(42);
  normal_distribution<double> transDist(0.0, transSigma);
  normal_distribution<double> rotDist(0.0, rotSigma);

  for (size_t trial = 0; trial < numTrials; ++trial) {
    typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;
    LevenbergMarquardtParams params;
    BatchFixedLagSmoother smootherOn(lag, params, true);
    BatchFixedLagSmoother smootherOff(lag, params, false);

    for (size_t i = 0; i <= numSteps; ++i) {
      NonlinearFactorGraph newFactors;
      Values newValues;
      Timestamps newTimestamps;

      Key key_i(i);

      if (i == 0) {
        newFactors.addPrior(key_i, groundTruth[0], noise);
        Pose2 initEst(groundTruth[0].x() + transDist(rng),
                      groundTruth[0].y() + transDist(rng),
                      groundTruth[0].theta() + rotDist(rng));
        newValues.insert(key_i, initEst);
      } else {
        // Noisy odometry measurement
        Pose2 noisyOdom(odomGT.x() + transDist(rng),
                        odomGT.y() + transDist(rng),
                        odomGT.theta() + rotDist(rng));
        newFactors.push_back(BetweenFactor<Pose2>(Key(i-1), key_i, noisyOdom, noise));

        // Initial estimate: perturbed ground truth
        Pose2 initEst(groundTruth[i].x() + transDist(rng) * 2,
                      groundTruth[i].y() + transDist(rng) * 2,
                      groundTruth[i].theta() + rotDist(rng) * 2);
        newValues.insert(key_i, initEst);
      }
      newTimestamps[key_i] = double(i);

      smootherOn.update(newFactors, newValues, newTimestamps);
      smootherOff.update(newFactors, newValues, newTimestamps);
    }

    // Compute NEES at the last key
    Key lastKey(numSteps);

    try {
      // enforceConsistency = true
      Values estOn = smootherOn.calculateEstimate();
      Marginals marginalsOn(smootherOn.getFactors(), estOn, Marginals::QR);
      Matrix covOn = marginalsOn.marginalCovariance(lastKey);
      Vector errOn = groundTruth[numSteps].localCoordinates(estOn.at<Pose2>(lastKey));
      neesSum_on += errOn.transpose() * covOn.inverse() * errOn;

      // enforceConsistency = false
      Values estOff = smootherOff.calculateEstimate();
      Marginals marginalsOff(smootherOff.getFactors(), estOff, Marginals::QR);
      Matrix covOff = marginalsOff.marginalCovariance(lastKey);
      Vector errOff = groundTruth[numSteps].localCoordinates(estOff.at<Pose2>(lastKey));
      neesSum_off += errOff.transpose() * covOff.inverse() * errOff;

      neesCount++;
    } catch (...) {
      continue;
    }
  }

  double avgNees_on = neesSum_on / neesCount;
  double avgNees_off = neesSum_off / neesCount;

  cout << "NEES Evaluation (" << neesCount << "/" << numTrials << " trials, Pose2):" << endl;
  cout << "  enforceConsistency=true  (FEJ): avg NEES = " << avgNees_on
       << " (expected: " << stateDim << ")" << endl;
  cout << "  enforceConsistency=false       : avg NEES = " << avgNees_off
       << " (expected: " << stateDim << ")" << endl;

  EXPECT(neesCount > 0);
  EXPECT(avgNees_on > 0.0);
  EXPECT(avgNees_off > 0.0);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
