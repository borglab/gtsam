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
hexagon = circlePose2(6,1.0);
p0 = hexagon.at(0);
p1 = hexagon.at(1);

%% create a Pose graph with one equality constraint and one measurement
fg = NonlinearFactorGraph;
fg.add(NonlinearEqualityPose2(0, p0));
delta = p0.between(p1);
covariance = noiseModel.Diagonal.Sigmas([0.05; 0.05; 5*pi/180]);
fg.add(BetweenFactorPose2(0,1, delta, covariance));
fg.add(BetweenFactorPose2(1,2, delta, covariance));
fg.add(BetweenFactorPose2(2,3, delta, covariance));
fg.add(BetweenFactorPose2(3,4, delta, covariance));
fg.add(BetweenFactorPose2(4,5, delta, covariance));
fg.add(BetweenFactorPose2(5,0, delta, covariance));

%% Create initial config
initial = Values;
initial.insert(0, p0);
initial.insert(1, hexagon.at(1).retract([-0.1, 0.1,-0.1]'));
initial.insert(2, hexagon.at(2).retract([ 0.1,-0.1, 0.1]'));
initial.insert(3, hexagon.at(3).retract([-0.1, 0.1,-0.1]'));
initial.insert(4, hexagon.at(4).retract([ 0.1,-0.1, 0.1]'));
initial.insert(5, hexagon.at(5).retract([-0.1, 0.1,-0.1]'));

%% Plot Initial Estimate
cla
plot2DTrajectory(initial, 'g*-'); axis equal

%% optimize
optimizer = DoglegOptimizer(fg, initial);
result = optimizer.optimizeSafely;

%% Show Result
hold on; plot2DTrajectory(result, 'b*-');
view(2);
axis([-1.5 1.5 -1.5 1.5]);
result.print(sprintf('\nFinal result:\n'));
