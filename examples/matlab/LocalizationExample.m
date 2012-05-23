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
priorNoise = gtsamSharedNoiseModel_Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.addPrior(1, priorMean, priorNoise); % add directly to graph

%% Add two odometry factors
odometry = gtsamPose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = gtsamSharedNoiseModel_Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
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

%% Query the marginals
marginals = graph.marginals(result);
x{1}=gtsamSymbol('x',1); P{1}=marginals.marginalCovariance(x{1}.key)
x{2}=gtsamSymbol('x',2); P{2}=marginals.marginalCovariance(x{2}.key)
x{3}=gtsamSymbol('x',3); P{3}=marginals.marginalCovariance(x{3}.key)

%% Plot Trajectory
figure(1)
clf
X=[];Y=[];
for i=1:3
   pose_i = result.pose(i);
   X=[X;pose_i.x];
   Y=[Y;pose_i.y];
end
plot(X,Y,'b*-');

%% Plot Covariance Ellipses
hold on
for i=1:3
   pose_i = result.pose(i);
   covarianceEllipse([pose_i.x;pose_i.y],P{i},'g')
end
