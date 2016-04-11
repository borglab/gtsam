from __future__ import print_function
import gtsam
import math
import numpy as np

def Vector3(x, y, z): return np.array([x, y, z])

# 1. Create a factor graph container and add factors to it
graph = gtsam.NonlinearFactorGraph()

# 2a. Add a prior on the first pose, setting it to the origin
# A prior factor consists of a mean and a noise model (covariance matrix)
priorNoise = gtsam.noiseModel.Diagonal.Sigmas(Vector3(0.3, 0.3, 0.1))
graph.add(gtsam.PriorFactorPose2(1, gtsam.Pose2(0, 0, 0), priorNoise))

# For simplicity, we will use the same noise model for odometry and loop closures
model = gtsam.noiseModel.Diagonal.Sigmas(Vector3(0.2, 0.2, 0.1))

# 2b. Add odometry factors
# Create odometry (Between) factors between consecutive poses
graph.add(gtsam.BetweenFactorPose2(1, 2, gtsam.Pose2(2, 0, 0), model))
graph.add(gtsam.BetweenFactorPose2(2, 3, gtsam.Pose2(2, 0, math.pi / 2), model))
graph.add(gtsam.BetweenFactorPose2(3, 4, gtsam.Pose2(2, 0, math.pi / 2), model))
graph.add(gtsam.BetweenFactorPose2(4, 5, gtsam.Pose2(2, 0, math.pi / 2), model))

# 2c. Add the loop closure constraint
# This factor encodes the fact that we have returned to the same pose. In real
# systems, these constraints may be identified in many ways, such as appearance-based
# techniques with camera images. We will use another Between Factor to enforce this constraint:
graph.add(gtsam.BetweenFactorPose2(5, 2, gtsam.Pose2(2, 0, math.pi / 2), model))
graph.print("\nFactor Graph:\n")  # print

# 3. Create the data structure to hold the initialEstimate estimate to the
# solution. For illustrative purposes, these have been deliberately set to incorrect values
initialEstimate = gtsam.Values()
initialEstimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
initialEstimate.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
initialEstimate.insert(3, gtsam.Pose2(4.1, 0.1, math.pi / 2))
initialEstimate.insert(4, gtsam.Pose2(4.0, 2.0, math.pi))
initialEstimate.insert(5, gtsam.Pose2(2.1, 2.1, -math.pi / 2))
initialEstimate.print("\nInitial Estimate:\n")  # print

# 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
# The optimizer accepts an optional set of configuration parameters,
# controlling things like convergence criteria, the type of linear
# system solver to use, and the amount of information displayed during
# optimization. We will set a few parameters as a demonstration.
parameters = gtsam.GaussNewtonParams()

# Stop iterating once the change in error between steps is less than this value
parameters.relativeErrorTol = 1e-5
# Do not perform more than N iteration steps
parameters.maxIterations = 100
# Create the optimizer ...
optimizer = gtsam.GaussNewtonOptimizer(graph, initialEstimate, parameters)
# ... and optimize
result = optimizer.optimize()
result.print("Final Result:\n")

# 5. Calculate and print marginal covariances for all variables
marginals = gtsam.Marginals(graph, result)
print("x1 covariance:\n", marginals.marginalCovariance(1))
print("x2 covariance:\n", marginals.marginalCovariance(2))
print("x3 covariance:\n", marginals.marginalCovariance(3))
print("x4 covariance:\n", marginals.marginalCovariance(4))
print("x5 covariance:\n", marginals.marginalCovariance(5))


