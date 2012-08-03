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

%% Create a hexagon of poses
import gtsam.*
hexagon = gtsam.circlePose3(6,1.0);
p0 = hexagon.at(0);
p1 = hexagon.at(1);

%% create a Pose graph with one equality constraint and one measurement
import gtsam.*
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
import gtsam.*
initial = Values;
s = 0.10;
initial.insert(0, p0);
initial.insert(1, hexagon.at(1).retract(s*randn(6,1)));
initial.insert(2, hexagon.at(2).retract(s*randn(6,1)));
initial.insert(3, hexagon.at(3).retract(s*randn(6,1)));
initial.insert(4, hexagon.at(4).retract(s*randn(6,1)));
initial.insert(5, hexagon.at(5).retract(s*randn(6,1)));

%% Plot Initial Estimate
import gtsam.*
cla
gtsam.plot3DTrajectory(initial, 'g-*');

%% optimize
import gtsam.*
optimizer = DoglegOptimizer(fg, initial);
result = optimizer.optimizeSafely();

%% Show Result
import gtsam.*
hold on; gtsam.plot3DTrajectory(result, 'b-*', true, 0.3);
axis([-2 2 -2 2 -1 1]);
axis equal
view(-37,40)
colormap hot
result.print(sprintf('\nFinal result:\n'));
