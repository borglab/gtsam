%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Create a hexagon of poses
hexagon = circlePose3(6,1.0);
p0 = hexagon.atPose3(0);
p1 = hexagon.atPose3(1);

%% create a Pose graph with one equality constraint and one measurement
fg = NonlinearFactorGraph;
fg.add(NonlinearEqualityPose3(0, p0));
delta = p0.between(p1);
covariance = noiseModel.Diagonal.Sigmas([0.05; 0.05; 0.05; 5*pi/180; 5*pi/180; 5*pi/180]);
fg.add(BetweenFactorPose3(0,1, delta, covariance));
fg.add(BetweenFactorPose3(1,2, delta, covariance));
fg.add(BetweenFactorPose3(2,3, delta, covariance));
fg.add(BetweenFactorPose3(3,4, delta, covariance));
fg.add(BetweenFactorPose3(4,5, delta, covariance));
fg.add(BetweenFactorPose3(5,0, delta, covariance));

%% Create initial config
initial = Values;
s = 0.10;
initial.insert(0, p0);
initial.insert(1, hexagon.atPose3(1).retract(s*randn(6,1)));
initial.insert(2, hexagon.atPose3(2).retract(s*randn(6,1)));
initial.insert(3, hexagon.atPose3(3).retract(s*randn(6,1)));
initial.insert(4, hexagon.atPose3(4).retract(s*randn(6,1)));
initial.insert(5, hexagon.atPose3(5).retract(s*randn(6,1)));

%% optimize
optimizer = LevenbergMarquardtOptimizer(fg, initial);
result = optimizer.optimizeSafely;

pose_1 = result.atPose3(1);
CHECK('pose_1.equals(Pose3,1e-4)',pose_1.equals(p1,1e-4));


