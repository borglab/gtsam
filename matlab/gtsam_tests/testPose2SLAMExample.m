%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Simple robotics example using the pre-built planar SLAM domain
% @author Alex Cunningham
% @author Frank Dellaert
% @author Chris Beall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have full odometry for measurements
%  - The robot is on a grid, moving 2 meters each step

%% Create graph container and add factors to it
graph = gtsam.NonlinearFactorGraph;

%% Add prior
% gaussian for prior
priorMean = gtsam.Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(gtsam.BetweenFactorPose2(1, 2, gtsam.Pose2(2.0, 0.0, 0.0), odometryNoise));
graph.add(gtsam.BetweenFactorPose2(2, 3, gtsam.Pose2(2.0, 0.0, pi/2), odometryNoise));
graph.add(gtsam.BetweenFactorPose2(3, 4, gtsam.Pose2(2.0, 0.0, pi/2), odometryNoise));
graph.add(gtsam.BetweenFactorPose2(4, 5, gtsam.Pose2(2.0, 0.0, pi/2), odometryNoise));

%% Add pose constraint
model = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(gtsam.BetweenFactorPose2(5, 2, gtsam.Pose2(2.0, 0.0, pi/2), model));

%% Initialize to noisy points
initialEstimate = gtsam.Values;
initialEstimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, gtsam.Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, gtsam.Pose2(4.1, 0.1, pi/2));
initialEstimate.insert(4, gtsam.Pose2(4.0, 2.0, pi));
initialEstimate.insert(5, gtsam.Pose2(2.1, 2.1,-pi/2));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();

%% Plot Covariance Ellipses
marginals = gtsam.Marginals(graph, result);
P = marginals.marginalCovariance(1);

pose_1 = result.atPose2(1);
gtsam.CHECK('pose_1.equals(Pose2,1e-4)',pose_1.equals(gtsam.Pose2,1e-4));
