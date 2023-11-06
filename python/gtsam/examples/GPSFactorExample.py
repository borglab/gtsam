"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robot motion example, with prior and one GPS measurements
Author: Mandy Xie
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import gtsam

# ENU Origin is where the plane was in hold next to runway
lat0 = 33.86998
lon0 = -84.30626
h0 = 274

# Create noise models
GPS_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
PRIOR_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 0.25)


def main():
    """Main runner."""
    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Add a prior on the first point, setting it to the origin
    # A prior factor consists of a mean and a noise model (covariance matrix)
    priorMean = gtsam.Pose3()  # prior at origin
    graph.add(gtsam.PriorFactorPose3(1, priorMean, PRIOR_NOISE))

    # Add GPS factors
    gps = gtsam.Point3(lat0, lon0, h0)
    graph.add(gtsam.GPSFactor(1, gps, GPS_NOISE))
    print("\nFactor Graph:\n{}".format(graph))

    # Create the data structure to hold the initialEstimate estimate to the solution
    # For illustrative purposes, these have been deliberately set to incorrect values
    initial = gtsam.Values()
    initial.insert(1, gtsam.Pose3())
    print("\nInitial Estimate:\n{}".format(initial))

    # optimize using Levenberg-Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))


if __name__ == "__main__":
    main()
