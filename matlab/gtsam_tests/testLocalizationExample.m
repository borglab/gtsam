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

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add two odometry factors
odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise));

%% Add three "GPS" measurements
% We use Pose2 Priors here with high variance on theta
groundTruth = Values;
groundTruth.insert(1, Pose2(0.0, 0.0, 0.0));
groundTruth.insert(2, Pose2(2.0, 0.0, 0.0));
groundTruth.insert(3, Pose2(4.0, 0.0, 0.0));
model = noiseModel.Diagonal.Sigmas([0.1; 0.1; 10]);
for i=1:3
    graph.add(PriorFactorPose2(i, groundTruth.at(i), model));
end

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();

%% Plot Covariance Ellipses
marginals = Marginals(graph, result);
P={};
for i=1:result.size()
    pose_i = result.at(i);
    CHECK('pose_i.equals(groundTruth.pose(i)',pose_i.equals(groundTruth.at(i),1e-4));
    P{i}=marginals.marginalCovariance(i);
end
