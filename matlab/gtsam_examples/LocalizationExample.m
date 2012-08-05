%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief Example of a simple 2D localization example
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add two odometry factors
odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise));

%% Add three "GPS" measurements
% We use Pose2 Priors here with high variance on theta
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 10]);
graph.add(PriorFactorPose2(1, Pose2(0.0, 0.0, 0.0), priorNoise));
graph.add(PriorFactorPose2(2, Pose2(2.0, 0.0, 0.0), priorNoise));
graph.add(PriorFactorPose2(3, Pose2(4.0, 0.0, 0.0), priorNoise));

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));

%% Plot trajectory and covariance ellipses
cla;
hold on;

plot2DTrajectory(result, [], Marginals(graph, result));

axis([-0.6 4.8 -1 1])
axis equal
view(2)
