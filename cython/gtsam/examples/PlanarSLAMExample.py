import gtsam
import numpy as np
import math

# Create an empty nonlinear factor graph
graph = gtsam.NonlinearFactorGraph()

# Create the keys we need for graph
# static Symbol x1('x',1), x2('x',2), x3('x',3);
# static Symbol l1('l',1), l2('l',2);
x1 = gtsam.symbol(ord('x'), 1)
x2 = gtsam.symbol(ord('x'), 2)
x3 = gtsam.symbol(ord('x'), 3)
l1 = gtsam.symbol(ord('l'), 4)
l2 = gtsam.symbol(ord('l'), 5)

# Add a prior on pose x1 at the origin. A prior factor consists of a mean and a noise model (covariance matrix)
# Pose2 prior(0.0, 0.0, 0.0); // prior mean is at origin
priorMean = gtsam.Pose2(0.0, 0.0, 0.0) # prior at origin
# noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
priorNoise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
# graph.emplace_shared<PriorFactor<Pose2> >(x1, prior, priorNoise); // add directly to graph
graph.add(gtsam.PriorFactorPose2(x1, priorMean, priorNoise))


# Add two odometry factors between x1,x2 and x2,x3
# Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
# noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
# For simplicity, we will use the same noise model for each odometry factor
odometryNoise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
# Create odometry (Between) factors between consecutive poses
# graph.emplace_shared<BetweenFactor<Pose2> >(x1, x2, odometry, odometryNoise);
graph.add(gtsam.BetweenFactorPose2(x1, x2, odometry, odometryNoise))
# graph.emplace_shared<BetweenFactor<Pose2> >(x2, x3, odometry, odometryNoise);
graph.add(gtsam.BetweenFactorPose2(x2, x3, odometry, odometryNoise))

# Add Range-Bearing measurements to two different landmarks
# create a noise model for the landmark measurements
# noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
measurementNoise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.1, 0.2]))
# Rot2 bearing11 = Rot2::fromDegrees(45),
# bearing21 = Rot2::fromDegrees(90),
# bearing32 = Rot2::fromDegrees(90);
# double range11 = std::sqrt(4.0+4.0),
# range21 = 2.0,
# range32 = 2.0;
bearing11 = gtsam.Rot2.fromDegrees(np.pi/4)
bearing21 = gtsam.Rot2.fromDegrees(np.pi/2)
bearing32 = gtsam.Rot2.fromDegrees(np.pi/2)
range11 = np.sqrt(4.0+4.0)
range21 = 2.0
range32 = 2.0

# Add Bearing-Range factors
# graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x1, l1, bearing11, range11, measurementNoise);
# graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x2, l1, bearing21, range21, measurementNoise);
# graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x3, l2, bearing32, range32, measurementNoise);
graph.add(gtsam.BearingRangeFactor2D(x1,l1,bearing11,range11,measurementNoise))
graph.add(gtsam.BearingRangeFactor2D(x2,l1,bearing21,range21,measurementNoise))
graph.add(gtsam.BearingRangeFactor2D(x3,l2,bearing32,range32,measurementNoise))

# Print graph
graph.print_("Factor Graph:\n");

# Create (deliberately inaccurate) initial estimate
# Values initialEstimate;
# initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
# initialEstimate.insert(x2, Pose2(2.3, 0.1,-0.2));
# initialEstimate.insert(x3, Pose2(4.1, 0.1, 0.1));
# initialEstimate.insert(l1, Point2(1.8, 2.1));
# initialEstimate.insert(l2, Point2(4.1, 1.8));
initialEstimate = gtsam.Values()
initialEstimate.insert(x1, gtsam.Pose2(-0.25, 0.20, 0.15))
initialEstimate.insert(x2, gtsam.Pose2( 2.30, 0.10, -0.20))
initialEstimate.insert(x3, gtsam.Pose2( 4.10, 0.10, 0.10))
initialEstimate.insert(l1, gtsam.Point2( 1.80, 2.10))
initialEstimate.insert(l2, gtsam.Point2( 4.10, 1.80))

# Print
initialEstimate.print_("Initial Estimate:\n");

# Optimize using Levenberg-Marquardt optimization. The optimizer
# accepts an optional set of configuration parameters, controlling
# things like convergence criteria, the type of linear system solver
# to use, and the amount of information displayed during optimization.
# Here we will use the default set of parameters.  See the
# documentation for the full set of parameters.
# LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
# Values result = optimizer.optimize();
# result.print("Final Result:\n");
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate, params)
result = optimizer.optimize()
result.print_("\nFinal Result:\n")

# Calculate and print marginal covariances for all variables
# Marginals marginals(graph, result);
# print(marginals.marginalCovariance(x1), "x1 covariance");
# print(marginals.marginalCovariance(x2), "x2 covariance");
# print(marginals.marginalCovariance(x3), "x3 covariance");
# print(marginals.marginalCovariance(l1), "l1 covariance");
# print(marginals.marginalCovariance(l2), "l2 covariance");
marginals = gtsam.Marginals(graph, result)
print("x1 covariance:\n", marginals.marginalCovariance(x1))
print("x2 covariance:\n", marginals.marginalCovariance(x2))
print("x3 covariance:\n", marginals.marginalCovariance(x3))
print("x4 covariance:\n", marginals.marginalCovariance(l1))
print("x5 covariance:\n", marginals.marginalCovariance(l2))
