%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Simple 2D robotics example using the SimpleSPCGSolver
% @author Yong-Dian Jian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have full odometry for measurements
%  - The robot is on a grid, moving 2 meters each step

%% Create graph container and add factors to it
graph = pose2SLAMGraph;

%% Add prior
% gaussian for prior
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsamnoiseModelDiagonal_Sigmas([0.3; 0.3; 0.1]);
graph.addPrior(1, priorMean, priorNoise); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = gtsamnoiseModelDiagonal_Sigmas([0.2; 0.2; 0.1]);
graph.addOdometry(1, 2, gtsamPose2(2.0, 0.0, 0.0 ), odometryNoise);
graph.addOdometry(2, 3, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);
graph.addOdometry(3, 4, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);
graph.addOdometry(4, 5, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);

%% Add pose constraint
model = gtsamnoiseModelDiagonal_Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(5, 2, gtsamPose2(2.0, 0.0, pi/2), model);

% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = pose2SLAMValues;
initialEstimate.insertPose(1, gtsamPose2(0.5, 0.0, 0.2 ));
initialEstimate.insertPose(2, gtsamPose2(2.3, 0.1,-0.2 ));
initialEstimate.insertPose(3, gtsamPose2(4.1, 0.1, pi/2));
initialEstimate.insertPose(4, gtsamPose2(4.0, 2.0, pi  ));
initialEstimate.insertPose(5, gtsamPose2(2.1, 2.1,-pi/2));
initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimizeSPCG(initialEstimate);
result.print(sprintf('\nFinal result:\n'));