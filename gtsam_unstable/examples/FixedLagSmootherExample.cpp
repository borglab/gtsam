/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedLagSmootherExample.cpp
 * @brief Demonstration of the fixed-lag smoothers using a planar robot example and multiple odometry-like sensors
 * @author Stephen Williams
 */

/**
 * A simple 2D pose slam example with multiple odometry-like measurements
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - The robot moves forward at 2m/s
 *  - We have measurements between each pose from multiple odometry sensors
 */

// This example demonstrates the use of the Fixed-Lag Smoothers in GTSAM unstable
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// We will use simple integer Keys to uniquely identify each robot pose.
#include <gtsam/inference/Key.h>

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

#include <iomanip>

using namespace std;
using namespace gtsam;


int main(int argc, char** argv) {

  // Define the smoother lag (in seconds)
  double lag = 2.0;

  // Create a fixed lag smoother
  // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
  BatchFixedLagSmoother smootherBatch(lag);
  // The Incremental version uses iSAM2 to perform the nonlinear optimization
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
  parameters.relinearizeSkip = 1; // Relinearize every time
  IncrementalFixedLagSmoother smootherISAM2(lag, parameters);

  // Create containers to store the factors and linearization points that
  // will be sent to the smoothers
  NonlinearFactorGraph newFactors;
  Values newValues;
  FixedLagSmoother::KeyTimestampMap newTimestamps;

  // Create a prior on the first pose, placing it at the origin
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  Key priorKey = 0;
  newFactors.addPrior(priorKey, priorMean, priorNoise);
  newValues.insert(priorKey, priorMean); // Initialize the first pose at the mean of the prior
  newTimestamps[priorKey] = 0.0; // Set the timestamp associated with this key to 0.0 seconds;

  // Now, loop through several time steps, creating factors from different "sensors"
  // and adding them to the fixed-lag smoothers
  double deltaT = 0.25;
  for(double time = deltaT; time <= 3.0; time += deltaT) {

    // Define the keys related to this timestamp
    Key previousKey(1000 * (time-deltaT));
    Key currentKey(1000 * (time));

    // Assign the current key to the current timestamp
    newTimestamps[currentKey] = time;

    // Add a guess for this pose to the new values
    // Since the robot moves forward at 2 m/s, then the position is simply: time[s]*2.0[m/s]
    // {This is not a particularly good way to guess, but this is just an example}
    Pose2 currentPose(time * 2.0, 0.0, 0.0);
    newValues.insert(currentKey, currentPose);

    // Add odometry factors from two different sources with different error stats
    Pose2 odometryMeasurement1 = Pose2(0.61, -0.08, 0.02);
    noiseModel::Diagonal::shared_ptr odometryNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
    newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement1, odometryNoise1));

    Pose2 odometryMeasurement2 = Pose2(0.47, 0.03, 0.01);
    noiseModel::Diagonal::shared_ptr odometryNoise2 = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
    newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement2, odometryNoise2));

    // Update the smoothers with the new factors. In this example, batch smoother needs one iteration
    // to accurately converge. The ISAM smoother doesn't, but we only start getting estiates when
    // both are ready for simplicity.
    if (time >= 0.50) {
      smootherBatch.update(newFactors, newValues, newTimestamps);
      smootherISAM2.update(newFactors, newValues, newTimestamps);
      for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
          smootherISAM2.update();
      }

      // Print the optimized current pose
      cout << setprecision(5) << "Timestamp = " << time << endl;
      smootherBatch.calculateEstimate<Pose2>(currentKey).print("Batch Estimate:");
      smootherISAM2.calculateEstimate<Pose2>(currentKey).print("iSAM2 Estimate:");
      cout << endl;

      // Clear contains for the next iteration
      newTimestamps.clear();
      newValues.clear();
      newFactors.resize(0);
    }
  }

  // And to demonstrate the fixed-lag aspect, print the keys contained in each smoother after 3.0 seconds
  cout << "After 3.0 seconds, " << endl;
  cout << "  Batch Smoother Keys: " << endl;
  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: smootherBatch.timestamps()) {
    cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
  }
  cout << "  iSAM2 Smoother Keys: " << endl;
  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: smootherISAM2.timestamps()) {
    cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
  }

  // Here is an example of how to get the full Jacobian of the problem.
  // First, get the linearization point.
  Values result = smootherISAM2.calculateEstimate();

  // Get the factor graph
  auto &factorGraph = smootherISAM2.getFactors();

  // Linearize to a Gaussian factor graph
  boost::shared_ptr<GaussianFactorGraph> linearGraph = factorGraph.linearize(result);

  // Converts the linear graph into a Jacobian factor and extracts the Jacobian matrix
  Matrix jacobian = linearGraph->jacobian().first;
  cout << " Jacobian: " << jacobian << endl;

  return 0;
}
