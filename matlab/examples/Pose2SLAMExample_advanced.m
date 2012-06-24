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
% @author Vadim Indelman
% @author Can Erdogan
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
priorNoise = gtsamnoiseModelDiagonal_Sigmas([0.3; 0.3; 0.1]);
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior at origin
graph.addPosePrior(1, priorMean, priorNoise); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = gtsamnoiseModelDiagonal_Sigmas([0.2; 0.2; 0.1]);
odometry = gtsamPose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
graph.addRelativePose(1, 2, odometry, odometryNoise);
graph.addRelativePose(2, 3, odometry, odometryNoise);

%% Add measurements
% general noisemodel for measurements
measurementNoise = gtsamnoiseModelDiagonal_Sigmas([0.1; 0.2]);

% print
graph.print('full graph');

%% Initialize to noisy points
initialEstimate = pose2SLAMValues;
initialEstimate.insertPose(1, gtsamPose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(2, gtsamPose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(3, gtsamPose2(4.1, 0.1, 0.1));

initialEstimate.print('initial estimate');

%% set up solver, choose ordering and optimize
%params = gtsamNonlinearOptimizationParameters_newDecreaseThresholds(1e-15, 1e-15);
%
%ord = graph.orderingCOLAMD(initialEstimate);
%
%result = pose2SLAMOptimizer(graph,initialEstimate,ord,params);                      
%result.print('final result');

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate,1);
result.print('final result');

%% Get the corresponding dense matrix
ord = graph.orderingCOLAMD(result);
gfg = graph.linearize(result,ord);
denseAb = gfg.denseJacobian;

%% Get sparse matrix A and RHS b
IJS = gfg.sparseJacobian_();
Ab=sparse(IJS(1,:),IJS(2,:),IJS(3,:));
A = Ab(:,1:end-1);
b = full(Ab(:,end));
spy(A);
