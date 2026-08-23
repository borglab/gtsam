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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
i1 = gtsam.symbol('x',1); i2 = gtsam.symbol('x',2); i3 = gtsam.symbol('x',3);
j1 = gtsam.symbol('l',1); j2 = gtsam.symbol('l',2);

%% Create graph container and add factors to it
graph = gtsam.NonlinearFactorGraph;

%% Add prior
priorMean = gtsam.Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(gtsam.PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph

%% Add odometry
odometry = gtsam.Pose2(2.0, 0.0, 0.0);
odometryNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(gtsam.BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(gtsam.BetweenFactorPose2(i2, i3, odometry, odometryNoise));

%% Add bearing/range measurement factors
degrees = pi/180;
brNoise = gtsam.noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(gtsam.BearingRangeFactor2D(i1, j1, gtsam.Rot2(45*degrees), sqrt(4+4), brNoise));
graph.add(gtsam.BearingRangeFactor2D(i2, j1, gtsam.Rot2(90*degrees), 2, brNoise));
graph.add(gtsam.BearingRangeFactor2D(i3, j2, gtsam.Rot2(90*degrees), 2, brNoise));

%% Initialize to noisy points
initialEstimate = gtsam.Values;
initialEstimate.insert(i1, gtsam.Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(i2, gtsam.Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(i3, gtsam.Pose2(4.1, 0.1, 0.1));
initialEstimate.insert(j1, gtsam.Point2(1.8, 2.1));
initialEstimate.insert(j2, gtsam.Point2(4.1, 1.8));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
marginals = gtsam.Marginals(graph, result);

%% Check first pose and point equality
pose_1 = result.atPose2(gtsam.symbol('x',1));
marginals.marginalCovariance(gtsam.symbol('x',1));
gtsam.CHECK('pose_1.equals(Pose2,1e-4)',pose_1.equals(gtsam.Pose2,1e-4));

point_1 = result.atPoint2(gtsam.symbol('l',1));
marginals.marginalCovariance(gtsam.symbol('l',1));
gtsam.CHECK('point_1.equals(Point2(2,2),1e-4)', ...
    norm(point_1 - gtsam.Point2(2,2)) < 1e-4);
