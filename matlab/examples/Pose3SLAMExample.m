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
hexagon = pose3SLAMValues.Circle(6,1.0);
p0 = hexagon.pose(0);
p1 = hexagon.pose(1);

%% create a Pose graph with one equality constraint and one measurement
fg = pose3SLAMGraph;
fg.addPoseConstraint(0, p0);
delta = p0.between(p1);
covariance = gtsamnoiseModelDiagonal.Sigmas([0.05; 0.05; 0.05; 5*pi/180; 5*pi/180; 5*pi/180]);
fg.addRelativePose(0,1, delta, covariance);
fg.addRelativePose(1,2, delta, covariance);
fg.addRelativePose(2,3, delta, covariance);
fg.addRelativePose(3,4, delta, covariance);
fg.addRelativePose(4,5, delta, covariance);
fg.addRelativePose(5,0, delta, covariance);

%% Create initial config
initial = pose3SLAMValues;
s = 0.10;
initial.insertPose(0, p0);
initial.insertPose(1, hexagon.pose(1).retract(s*randn(6,1)));
initial.insertPose(2, hexagon.pose(2).retract(s*randn(6,1)));
initial.insertPose(3, hexagon.pose(3).retract(s*randn(6,1)));
initial.insertPose(4, hexagon.pose(4).retract(s*randn(6,1)));
initial.insertPose(5, hexagon.pose(5).retract(s*randn(6,1)));

%% Plot Initial Estimate
cla
T=initial.translations();
plot3(T(:,1),T(:,2),T(:,3),'g-*');

%% optimize
result = fg.optimize(initial,1);

%% Show Result
hold on; plot3DTrajectory(result,'b-*', true, 0.3);
axis([-2 2 -2 2 -1 1]);
axis equal
view(-37,40)
colormap hot
result.print(sprintf('\nFinal result:\n'));
