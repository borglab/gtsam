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
import gtsam.*
graph = NonlinearFactorGraph;

%% Add a Gaussian prior on pose x_1
import gtsam.*
priorMean = Pose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add two odometry factors
import gtsam.*
odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise));

%% Initialize to noisy points
import gtsam.*
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
import gtsam.*
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
marginals = Marginals(graph, result);
marginals.marginalCovariance(1);

%% Check first pose equality
import gtsam.*
pose_1 = result.at(1);
CHECK('pose_1.equals(Pose2,1e-4)',pose_1.equals(Pose2,1e-4));
