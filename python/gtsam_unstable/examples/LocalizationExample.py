"""
A simple 2D pose slam example with "GPS" measurements
  - The robot moves forward 2 meter each iteration
  - The robot initially faces along the X axis (horizontal, to the right in 2D)
  - We have full odometry between pose
  - We have "GPS-like" measurements implemented with a custom factor
"""
import numpy as np

import gtsam
from gtsam import BetweenFactorPose2, Pose2, noiseModel
from gtsam_unstable import PartialPriorFactorPose2


def main():
    # 1. Create a factor graph container and add factors to it.
    graph = gtsam.NonlinearFactorGraph()

    # 2a. Add odometry factors
    # For simplicity, we will use the same noise model for each odometry factor
    odometryNoise = noiseModel.Diagonal.Sigmas(np.asarray([0.2, 0.2, 0.1]))

    # Create odometry (Between) factors between consecutive poses
    graph.push_back(
        BetweenFactorPose2(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise))
    graph.push_back(
        BetweenFactorPose2(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise))

    # 2b. Add "GPS-like" measurements
    # We will use PartialPrior factor for this.
    unaryNoise = noiseModel.Diagonal.Sigmas(np.array([0.1,
                                                      0.1]))  # 10cm std on x,y

    graph.push_back(
        PartialPriorFactorPose2(1, [0, 1], np.asarray([0.0, 0.0]), unaryNoise))
    graph.push_back(
        PartialPriorFactorPose2(2, [0, 1], np.asarray([2.0, 0.0]), unaryNoise))
    graph.push_back(
        PartialPriorFactorPose2(3, [0, 1], np.asarray([4.0, 0.0]), unaryNoise))
    graph.print("\nFactor Graph:\n")

    # 3. Create the data structure to hold the initialEstimate estimate to the solution
    # For illustrative purposes, these have been deliberately set to incorrect values
    initialEstimate = gtsam.Values()
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2))
    initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2))
    initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1))
    initialEstimate.print("\nInitial Estimate:\n")

    # 4. Optimize using Levenberg-Marquardt optimization. The optimizer
    # accepts an optional set of configuration parameters, controlling
    # things like convergence criteria, the type of linear system solver
    # to use, and the amount of information displayed during optimization.
    # Here we will use the default set of parameters.  See the
    # documentation for the full set of parameters.
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
    result = optimizer.optimize()
    result.print("Final Result:\n")

    # 5. Calculate and print marginal covariances for all variables
    marginals = gtsam.Marginals(graph, result)
    print("x1 covariance:\n", marginals.marginalCovariance(1))
    print("x2 covariance:\n", marginals.marginalCovariance(2))
    print("x3 covariance:\n", marginals.marginalCovariance(3))


if __name__ == "__main__":
    main()
