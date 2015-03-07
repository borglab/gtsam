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

import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2);

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph

%% Add odometry
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise));

%% Add bearing/range measurement factors
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(4+4), brNoise));
graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
graph.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(i1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(i2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(i3, Pose2(4.1, 0.1, 0.1));
initialEstimate.insert(j1, Point2(1.8, 2.1));
initialEstimate.insert(j2, Point2(4.1, 1.8));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
marginals = Marginals(graph, result);

%% Check first pose and point equality
pose_1 = result.at(symbol('x',1));
marginals.marginalCovariance(symbol('x',1));
CHECK('pose_1.equals(Pose2,1e-4)',pose_1.equals(Pose2,1e-4));

point_1 = result.at(symbol('l',1));
marginals.marginalCovariance(symbol('l',1));
CHECK('point_1.equals(Point2(2,2),1e-4)',point_1.equals(Point2(2,2),1e-4));


