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

import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have full odometry for measurements
%  - The robot is on a grid, moving 2 meters each step

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
% gaussian for prior
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(1, 2, Pose2(2.0, 0.0, 0.0 ), odometryNoise));
graph.add(BetweenFactorPose2(2, 3, Pose2(2.0, 0.0, pi/2), odometryNoise));
graph.add(BetweenFactorPose2(3, 4, Pose2(2.0, 0.0, pi/2), odometryNoise));
graph.add(BetweenFactorPose2(4, 5, Pose2(2.0, 0.0, pi/2), odometryNoise));

%% Add pose constraint
model = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(5, 2, Pose2(2.0, 0.0, pi/2), model));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2 ));
initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2 ));
initialEstimate.insert(3, Pose2(4.1, 0.1, pi/2));
initialEstimate.insert(4, Pose2(4.0, 2.0, pi  ));
initialEstimate.insert(5, Pose2(2.1, 2.1,-pi/2));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();

%% Plot Covariance Ellipses
marginals = Marginals(graph, result);
P = marginals.marginalCovariance(1);

pose_1 = result.at(1);
CHECK('pose_1.equals(Pose2,1e-4)',pose_1.equals(Pose2,1e-4));


