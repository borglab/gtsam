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
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise));

%% Add measurements
% general noisemodel for measurements
measurementNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);

% print
graph.print('full graph');

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));

initialEstimate.print('initial estimate');

%% set up solver, choose ordering and optimize
params = DoglegParams;
params.setAbsoluteErrorTol(1e-15);
params.setRelativeErrorTol(1e-15);
params.setVerbosity('ERROR');
params.setVerbosityDL('VERBOSE');
params.setOrdering(graph.orderingCOLAMD());
optimizer = DoglegOptimizer(graph, initialEstimate, params);                      

result = optimizer.optimizeSafely();
result.print('final result');

%% Get the corresponding dense matrix
ord = graph.orderingCOLAMD();
gfg = graph.linearize(result);
denseAb = gfg.augmentedJacobian;

%% Get sparse matrix A and RHS b
IJS = gfg.sparseJacobian_();
Ab=sparse(IJS(1,:),IJS(2,:),IJS(3,:));
A = Ab(:,1:end-1);
b = full(Ab(:,end));
clf
spy(A);
title('Non-zero entries in measurement Jacobian');
