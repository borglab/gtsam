/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2025, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
*  @file   IncrementalFixedLagExample.cpp
*  @brief  Example of incremental fixed-lag smoother using real-world data.
*  @author Xiangcheng Hu (xhubd@connect.ust.hk), Frank Dellaert, Kevin Doherty
*  @date   Janaury 15, 2025
*
* Key objectives:
*   - Validate `IncrementalFixedLagSmoother` functionality with real-world data.
*   - Showcase how setting `findUnusedFactorSlots = true` addresses the issue #1452 in GTSAM, ensuring
*     that unused factor slots (nullptrs) are correctly released when old factors are marginalized.
*
*  This example leverages pose measurements from a real scenario. The test data (named "IncrementalFixedLagSmootherTestData.txt") is
*  based on the corridor_day sequence from the FusionPortable dataset (https://fusionportable.github.io/dataset/fusionportable/).
*   - 1 prior factor derived from point cloud ICP alignment with a prior map.
*   - 199 relative pose factors derived from FAST-LIO2 odometry.
*
*  Data Format (IncrementalFixedLagSmootherTestData.txt):
*    1) PRIOR factor line:
*       @code
*       0 timestamp key x y z roll pitch yaw cov_6x6
*       @endcode
*       - "0" indicates PRIOR factor.
*       - "timestamp" is the observation time (in seconds).
*       - "key" is the integer ID for the Pose3 variable.
*       - (x, y, z, roll, pitch, yaw) define the pose.
*       - "cov_6x6" is the full 6x6 covariance matrix (row-major).
*
*    2) BETWEEN factor line:
*       @code
*       1 timestamp key1 key2 x y z roll pitch yaw cov_6x6
*       @endcode
*       - "1" indicates BETWEEN factor.
*       - "timestamp" is the observation time (in seconds).
*       - "key1" and "key2" are the integer IDs for the connected Pose3 variables.
*       - (x, y, z, roll, pitch, yaw) define the relative pose between these variables.
*       - "cov_6x6" is the full 6x6 covariance matrix (row-major).
*
*  See also:
*   - GTSAM Issue #1452: https://github.com/borglab/gtsam/issues/1452
*/

// STL
#include <iostream>
#include <string>
// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>  // for writeG2o

using namespace std;
using namespace gtsam;
// Shorthand for symbols
using symbol_shorthand::X;  // Pose3 (x,y,z, roll, pitch, yaw)

// Factor types
enum FactorType {
    PRIOR = 0,
    BETWEEN = 1
};

typedef Eigen::Matrix<double, 6, 6> Matrix6;

/* ************************************************************************* */
/**
 * @brief Read a 6x6 covariance matrix from an input string stream.
 *
 * @param iss Input string stream containing covariance entries.
 * @return 6x6 covariance matrix.
 */
Matrix6 readCovarianceMatrix(istringstream &iss) {
    Matrix6 cov;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            iss >> cov(r, c);
        }
    }
    return cov;
}

/* ************************************************************************* */
/**
 * @brief Create a Pose3 object from individual pose parameters.
 *
 * @param x     Translation in x
 * @param y     Translation in y
 * @param z     Translation in z
 * @param roll  Rotation about x-axis
 * @param pitch Rotation about y-axis
 * @param yaw   Rotation about z-axis
 * @return Constructed Pose3 object
 */
Pose3 createPose(double x, double y, double z, double roll, double pitch, double yaw) {
    return Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
}

/* ************************************************************************* */
/**
 * @brief Save the factor graph and estimates to a .g2o file (for visualization/debugging).
 *
 * @param graph       The factor graph
 * @param estimate    Current estimates of all variables
 * @param filename    Base filename for saving
 * @param iterCount   Iteration count to differentiate successive outputs
 */
void saveG2oGraph(const NonlinearFactorGraph &graph, const Values &estimate,
                  const string &filename, int iterCount) {
    // Create zero-padded iteration count
    string countStr = to_string(iterCount);
    string paddedCount = string(5 - countStr.length(), '0') + countStr;
    string fullFilename = filename + "_" + paddedCount + ".g2o";
    // Write graph and estimates to g2o file
    writeG2o(graph, estimate, fullFilename);
    cout << "\nSaved graph to: " << fullFilename << endl;
}

/* ************************************************************************* */
/**
 * @brief Main function: Reads poses data from a text file and performs incremental fixed-lag smoothing.
 *
 * Data Flow:
 *  1) Parse lines from "IncrementalFixedLagSmootherTestData.txt".
 *  2) For each line:
 *      - If it's a PRIOR factor, add a PriorFactor with a specified pose and covariance.
 *      - If it's a BETWEEN factor, add a BetweenFactor with a relative pose and covariance.
 *      - Insert new variables with initial guesses into the current solution if they don't exist.
 *  3) Update the fixed-lag smoother (with iSAM2 inside) to incrementally optimize and marginalize out old poses
 *     beyond the specified lag window.
 *  4) Repeat until all lines are processed.
 *  5) Save the resulting factor graph and estimate of the last sliding window to a .g2o file.
 */
int main() {
    string factor_loc = findExampleDataFile("issue1452.txt");
    ifstream factor_file(factor_loc.c_str());
    cout << "Reading factors data file: " << factor_loc << endl;

    // Configure ISAM2 parameters for the fixed-lag smoother
    ISAM2Params isamParameters;
    isamParameters.relinearizeThreshold = 0.1;
    isamParameters.relinearizeSkip = 1;

    // Important!!!!!! Key parameter to ensure old factors are released after marginalization
    isamParameters.findUnusedFactorSlots = true;
    // Initialize fixed-lag smoother with a 1-second lag window
    const double lag = 1.0;
    IncrementalFixedLagSmoother smoother(lag, isamParameters);
    // Print the iSAM2 parameters (optional)
    isamParameters.print();

    // Containers for incremental updates
    NonlinearFactorGraph newFactors;
    Values newValues;
    FixedLagSmoother::KeyTimestampMap newTimestamps;
    // For tracking the latest estimate of all states in the sliding window
    Values currentEstimate;
    Pose3 lastPose;

    // Read and process each line in the factor graph file
    string line;
    int lineCount = 0;
    while (getline(factor_file, line)) {
        if (line.empty()) continue;  // Skip empty lines
        cout << "\n======================== Processing line " << ++lineCount
             << " =========================" << endl;

        istringstream iss(line);
        int factorType;
        iss >> factorType;
        // Check if the factor is PRIOR or BETWEEN
        if (factorType == PRIOR) {
            /**
             * Format: PRIOR factor
             * factor_type timestamp key pose(x y z roll pitch yaw) cov(6x6)
             */
            double timestamp;
            int key;
            double x, y, z, roll, pitch, yaw;
            iss >> timestamp >> key >> x >> y >> z >> roll >> pitch >> yaw;
            Pose3 pose = createPose(x, y, z, roll, pitch, yaw);
            Matrix6 cov = readCovarianceMatrix(iss);
            auto noise = noiseModel::Gaussian::Covariance(cov);
            // Add prior factor
            newFactors.addPrior(X(key), pose, noise);
            cout << "Add PRIOR factor on key " << key << endl;
            // Provide initial guess if not already in the graph
            if (!newValues.exists(X(key))) {
                newValues.insert(X(key), pose);
                newTimestamps[X(key)] = timestamp;
            }
        } else if (factorType == BETWEEN) {
            /**
             * Format: BETWEEN factor
             * factor_type timestamp key1 key2 pose(x y z roll pitch yaw) cov(6x6)
             */
            double timestamp;
            int key1, key2;
            iss >> timestamp >> key1 >> key2;
            double x1, y1, z1, roll1, pitch1, yaw1;
            iss >> x1 >> y1 >> z1 >> roll1 >> pitch1 >> yaw1;
            Pose3 relativePose = createPose(x1, y1, z1, roll1, pitch1, yaw1);
            Matrix6 cov = readCovarianceMatrix(iss);
            auto noise = noiseModel::Gaussian::Covariance(cov);
            // Add between factor
            newFactors.emplace_shared<BetweenFactor<Pose3>>(X(key1), X(key2), relativePose, noise);
            cout << "Add BETWEEN factor: " << key1 << " -> " << key2 << endl;
            // Provide an initial guess if needed
            if (!newValues.exists(X(key2))) {
                newValues.insert(X(key2), lastPose.compose(relativePose));
                newTimestamps[X(key2)] = timestamp;
            }
        } else {
            cerr << "Unknown factor type: " << factorType << endl;
            continue;
        }

        // Print some intermediate statistics
        cout << "Before update - Graph has " << smoother.getFactors().size()
             << " factors, " << smoother.getFactors().nrFactors() << " nr factors." << endl;
        cout << "New factors: " << newFactors.size()
             << ", New values: " << newValues.size() << endl;

        // Attempt to update the smoother with new factors and values
        try {
            smoother.update(newFactors, newValues, newTimestamps);
            // Optional: Perform extra internal iterations if needed
            size_t maxExtraIterations = 3;
            for (size_t i = 1; i < maxExtraIterations; ++i) {
                smoother.update();
            }
            // you may not get expected results if you use the gtsam version lower than 4.3
            cout << "After update - Graph has " << smoother.getFactors().size()
                 << " factors, " << smoother.getFactors().nrFactors() << " nr factors." << endl;

            // Retrieve the latest estimate
            currentEstimate = smoother.calculateEstimate();
            if (!currentEstimate.empty()) {
                // Update lastPose to the last key's estimate
                Key lastKey = currentEstimate.keys().back();
                lastPose = currentEstimate.at<Pose3>(lastKey);
            }

            // Clear containers for the next iteration
            newFactors.resize(0);
            newValues.clear();
            newTimestamps.clear();
        } catch (const exception &e) {
            cerr << "Smoother update failed: " << e.what() << endl;
        }
    }

    // The results of the last sliding window are saved to a g2o file for visualization or further analysis.
    saveG2oGraph(smoother.getFactors(), currentEstimate, "isam", lineCount);
    factor_file.close();

    return 0;
}

