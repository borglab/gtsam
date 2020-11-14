%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Basic VO Example with 3 landmarks and two cameras
% @author Chris Beall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Assumptions
%  - For simplicity this example is in the camera's coordinate frame
%  - X: right, Y: down, Z: forward
%  - Pose x1 is at the origin, Pose 2 is 1 meter forward (along Z-axis)
%  - x1 is fixed with a constraint, x2 is initialized with noisy values
%  - No noise on measurements

%% Create keys for variables
x1 = symbol('x',1); x2 = symbol('x',2); 
l1 = symbol('l',1); l2 = symbol('l',2); l3 = symbol('l',3);

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% add a constraint on the starting pose
first_pose = Pose3();
graph.add(NonlinearEqualityPose3(x1, first_pose));

%% Create realistic calibration and measurement noise model
% format: fx fy skew cx cy baseline
K = Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2);
stereo_model = noiseModel.Isotropic.Sigma(3,1);

%% Add measurements
% pose 1
graph.add(GenericStereoFactor3D(StereoPoint2(520, 480, 440), stereo_model, x1, l1, K));
graph.add(GenericStereoFactor3D(StereoPoint2(120,  80, 440), stereo_model, x1, l2, K));
graph.add(GenericStereoFactor3D(StereoPoint2(320, 280, 140), stereo_model, x1, l3, K));

%pose 2
graph.add(GenericStereoFactor3D(StereoPoint2(570, 520, 490), stereo_model, x2, l1, K));
graph.add(GenericStereoFactor3D(StereoPoint2( 70,  20, 490), stereo_model, x2, l2, K));
graph.add(GenericStereoFactor3D(StereoPoint2(320, 270, 115), stereo_model, x2, l3, K));


%% Create initial estimate for camera poses and landmarks
initialEstimate = Values;
initialEstimate.insert(x1, first_pose);
% noisy estimate for pose 2
initialEstimate.insert(x2, Pose3(Rot3(), [0.1, -.1, 1.1]'));
initialEstimate.insert(l1, [ 1,  1, 5]');
initialEstimate.insert(l2, [-1,  1, 5]');
initialEstimate.insert(l3, [ 0,-.5, 5]');

%% optimize
fprintf(1,'Optimizing\n'); tic
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
toc

%% visualize initial trajectory, final trajectory, and final points
cla; hold on;
axis normal
axis([-1.5 1.5 -2 2 -1 6]);
axis equal
view(-38,12)
camup([0;1;0]);

plot3DTrajectory(initialEstimate, 'r', 1, 0.3);
plot3DTrajectory(result, 'g', 1, 0.3);
plot3DPoints(result);
