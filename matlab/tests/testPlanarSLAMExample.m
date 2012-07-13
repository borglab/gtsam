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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2);

%% Create graph container and add factors to it
graph = planarSLAMGraph;

%% Add prior
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsamnoiseModelDiagonal.Sigmas([0.3; 0.3; 0.1]);
graph.addPosePrior(i1, priorMean, priorNoise); % add directly to graph

%% Add odometry
odometry = gtsamPose2(2.0, 0.0, 0.0);
odometryNoise = gtsamnoiseModelDiagonal.Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(i1, i2, odometry, odometryNoise);
graph.addRelativePose(i2, i3, odometry, odometryNoise);

%% Add bearing/range measurement factors
degrees = pi/180;
noiseModel = gtsamnoiseModelDiagonal.Sigmas([0.1; 0.2]);
graph.addBearingRange(i1, j1, gtsamRot2(45*degrees), sqrt(4+4), noiseModel);
graph.addBearingRange(i2, j1, gtsamRot2(90*degrees), 2, noiseModel);
graph.addBearingRange(i3, j2, gtsamRot2(90*degrees), 2, noiseModel);

%% Initialize to noisy points
initialEstimate = planarSLAMValues;
initialEstimate.insertPose(i1, gtsamPose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(i2, gtsamPose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(i3, gtsamPose2(4.1, 0.1, 0.1));
initialEstimate.insertPoint(j1, gtsamPoint2(1.8, 2.1));
initialEstimate.insertPoint(j2, gtsamPoint2(4.1, 1.8));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate,0);
marginals = graph.marginals(result);

%% Check first pose and point equality
pose_1 = result.pose(symbol('x',1));
marginals.marginalCovariance(symbol('x',1));
CHECK('pose_1.equals(gtsamPose2,1e-4)',pose_1.equals(gtsamPose2,1e-4));

point_1 = result.point(symbol('l',1));
marginals.marginalCovariance(symbol('l',1));
CHECK('point_1.equals(gtsamPoint2(2,2),1e-4)',point_1.equals(gtsamPoint2(2,2),1e-4));


