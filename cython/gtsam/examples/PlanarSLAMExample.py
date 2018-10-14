"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robotics example using odometry measurements and bearing-range (laser) measurements
Author: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""

import numpy as np

from gtsam import (BearingRangeFactor2D, BetweenFactorPose2,
                   LevenbergMarquardtOptimizer, LevenbergMarquardtParams,
                   Marginals, NonlinearFactorGraph, Point2, Pose2,
                   PriorFactorPose2, Rot2, Values, noiseModel_Diagonal, symbol)

# Create noise models
PRIOR_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
ODOMETRY_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
MEASUREMENT_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.1, 0.2]))

# Create an empty nonlinear factor graph
graph = NonlinearFactorGraph()

# Create the keys corresponding to unknown variables in the factor graph
X1 = symbol(ord('x'), 1)
X2 = symbol(ord('x'), 2)
X3 = symbol(ord('x'), 3)
L1 = symbol(ord('l'), 4)
L2 = symbol(ord('l'), 5)

# Add a prior on pose X1 at the origin. A prior factor consists of a mean and a noise model
graph.add(PriorFactorPose2(X1, Pose2(0.0, 0.0, 0.0), PRIOR_NOISE))

# Add odometry factors between X1,X2 and X2,X3, respectively
graph.add(BetweenFactorPose2(X1, X2, Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))
graph.add(BetweenFactorPose2(X2, X3, Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))

# Add Range-Bearing measurements to two different landmarks L1 and L2
graph.add(BearingRangeFactor2D(
    X1, L1, Rot2.fromDegrees(45), np.sqrt(4.0+4.0), MEASUREMENT_NOISE))
graph.add(BearingRangeFactor2D(
    X2, L1, Rot2.fromDegrees(90), 2.0, MEASUREMENT_NOISE))
graph.add(BearingRangeFactor2D(
    X3, L2, Rot2.fromDegrees(90), 2.0, MEASUREMENT_NOISE))

# Print graph
graph.print_("Factor Graph:\n")

# Create (deliberately inaccurate) initial estimate
initial_estimate = Values()
initial_estimate.insert(X1, Pose2(-0.25, 0.20, 0.15))
initial_estimate.insert(X2, Pose2(2.30, 0.10, -0.20))
initial_estimate.insert(X3, Pose2(4.10, 0.10, 0.10))
initial_estimate.insert(L1, Point2(1.80, 2.10))
initial_estimate.insert(L2, Point2(4.10, 1.80))

# Print
initial_estimate.print_("Initial Estimate:\n")

# Optimize using Levenberg-Marquardt optimization. The optimizer
# accepts an optional set of configuration parameters, controlling
# things like convergence criteria, the type of linear system solver
# to use, and the amount of information displayed during optimization.
# Here we will use the default set of parameters.  See the
# documentation for the full set of parameters.
params = LevenbergMarquardtParams()
optimizer = LevenbergMarquardtOptimizer(graph, initial_estimate, params)
result = optimizer.optimize()
result.print_("\nFinal Result:\n")

# Calculate and print marginal covariances for all variables
marginals = Marginals(graph, result)
for (key, str) in [(X1,"X1"),(X2,"X2"),(X3,"X3"),(L1,"L1"),(L2,"L2")]:
    print("{} covariance:\n{}\n".format(str,marginals.marginalCovariance(key)))
