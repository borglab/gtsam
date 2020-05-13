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

% Copied Original file. Modified by David Jensen to use Pose3 instead of
% Pose2. Everything else is the same.

import gtsam.*

%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add a Gaussian prior on pose x_1
priorMean = Pose3();%Pose3.Expmap([0.0; 0.0; 0.0; 0.0; 0.0; 0.0]); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.3; 0.3; 0.3]); % 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
graph.add(PriorFactorPose3(1, priorMean, priorNoise)); % add directly to graph

%% Add two odometry factors
odometry = Pose3.Expmap([0.0; 0.0; 0.0; 2.0; 0.0; 0.0]); % create a measurement for both factors (the same in this case)
odometryNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.2; 0.2; 0.2]); % 20cm std on x,y,z 0.1 rad on roll,pitch,yaw
graph.add(BetweenFactorPose3(1, 2, odometry, odometryNoise));
graph.add(BetweenFactorPose3(2, 3, odometry, odometryNoise));

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
%initialEstimate.insert(1, Pose3.Expmap([0.2; 0.1; 0.1; 0.5; 0.0; 0.0]));
%initialEstimate.insert(2, Pose3.Expmap([-0.2; 0.1; -0.1; 2.3; 0.1; 0.1]));
%initialEstimate.insert(3, Pose3.Expmap([0.1; -0.1; 0.1; 4.1; 0.1; -0.1]));
%initialEstimate.print(sprintf('\nInitial estimate:\n  '));

for i=1:3
  deltaPosition = 0.5*rand(3,1) + [1;0;0]; % create random vector with mean = [1 0 0] and sigma = 0.5
  deltaRotation = 0.1*rand(3,1) + [0;0;0]; % create random rotation with mean [0 0 0] and sigma = 0.1 (rad)
  deltaPose = Pose3.Expmap([deltaRotation; deltaPosition]);
  currentPose = currentPose.compose(deltaPose);
  initialEstimate.insert(i, currentPose);
end

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));

%% Plot trajectory and covariance ellipses
cla;
hold on;

plot3DTrajectory(result, [], Marginals(graph, result));

axis([-0.6 4.8 -1 1])
axis equal
view(2)
