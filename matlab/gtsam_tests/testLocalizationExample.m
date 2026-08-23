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
graph = gtsam.NonlinearFactorGraph;

%% Add two odometry factors
odometry = gtsam.Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometryNoise));
graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, odometryNoise));

%% Add three "GPS" measurements
% We use Pose2 Priors here with high variance on theta
groundTruth = gtsam.Values;
groundTruth.insert(1, gtsam.Pose2(0.0, 0.0, 0.0));
groundTruth.insert(2, gtsam.Pose2(2.0, 0.0, 0.0));
groundTruth.insert(3, gtsam.Pose2(4.0, 0.0, 0.0));
model = gtsam.noiseModel.Diagonal.Sigmas([0.1; 0.1; 10]);
for i=1:3
    graph.add(gtsam.PriorFactorPose2(i, groundTruth.atPose2(i), model));
end

%% Initialize to noisy points
initialEstimate = gtsam.Values;
initialEstimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(2, gtsam.Pose2(2.3, 0.1,-0.2));
initialEstimate.insert(3, gtsam.Pose2(4.1, 0.1, 0.1));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();

%% Plot Covariance Ellipses
marginals = gtsam.Marginals(graph, result);
P={};
for i=1:result.size()
    pose_i = result.atPose2(i);
    gtsam.CHECK('pose_i.equals(groundTruth.pose(i)',pose_i.equals(groundTruth.atPose2(i),1e-4));
    P{i}=marginals.marginalCovariance(i);
end
