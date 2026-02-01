"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robot motion example, with prior and two odometry measurements
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np

# Create noise models
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


def main():
    """Main runner"""
    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Add a prior on the first pose, setting it to the origin
    # A prior factor consists of a mean and a noise model (covariance matrix)
    priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
    graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))

    # Add odometry factors
    odometry = gtsam.Pose2(2.0, 0.0, 0.0)
    # For simplicity, we will use the same noise model for each odometry factor
    # Create odometry (Between) factors between consecutive poses
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
    print("\nFactor Graph:\n{}".format(graph))

    # Create the data structure to hold the initialEstimate estimate to the solution
    # For illustrative purposes, these have been deliberately set to incorrect values
    initial = gtsam.Values()
    initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
    initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))
    print("\nInitial Estimate:\n{}".format(initial))

    # optimize using Levenberg-Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    # 5. Calculate and print marginal covariances for all variables
    marginals = gtsam.Marginals(graph, result)
    for i in range(1, 4):
        print("X{} covariance:\n{}\n".format(i,
                                             marginals.marginalCovariance(i)))

    for i in range(1, 4):
        gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5,
                              marginals.marginalCovariance(i))
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    main()
