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

%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = pose2SLAMGraph;

%% Add a Gaussian prior on pose x_1
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = gtsamnoiseModelDiagonal_Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.addPrior(1, priorMean, priorNoise); % add directly to graph

%% Add two odometry factors
odometry = gtsamPose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = gtsamnoiseModelDiagonal_Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
graph.addOdometry(1, 2, odometry, odometryNoise);
graph.addOdometry(2, 3, odometry, odometryNoise);

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = pose2SLAMValues;
initialEstimate.insertPose(1, gtsamPose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(2, gtsamPose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(3, gtsamPose2(4.1, 0.1, 0.1));
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate);
result.print(sprintf('\nFinal result:\n  '));

%% Plot Covariance Ellipses
cla;
plot(result.xs(),result.ys(),'k*-'); hold on
marginals = graph.marginals(result);
P={};
for i=1:result.size()
    pose_i = result.pose(i);
    P{i}=marginals.marginalCovariance(i);
    plotPose2(pose_i,'g',P{i})
end
axis([-0.6 4.8 -1 1])
axis equal
view(2)