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

%% Initialize to noisy points
initialEstimate = pose2SLAMValues;
initialEstimate.insertPose(1, gtsamPose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(2, gtsamPose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(3, gtsamPose2(4.1, 0.1, 0.1));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate);
marginals = graph.marginals(result);
marginals.marginalCovariance(i);

%% Check first pose equality
pose_i = result.pose(1);
CHECK('pose_1.equals(gtsamPose2,1e-4)',pose_1.equals(gtsamPose2,1e-4));