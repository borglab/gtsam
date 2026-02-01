%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read Stereo Visual Odometry from file and optimize
% @author Chris Beall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Load calibration
% format: fx fy skew cx cy baseline
calib = dlmread(findExampleDataFile('VO_calibration.txt'));
K = Cal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
stereo_model = noiseModel.Diagonal.Sigmas([1.0; 1.0; 1.0]);

%% create empty graph and values
graph = NonlinearFactorGraph;
initial = Values;


%% load the initial poses from VO
% row format: camera_id 4x4 pose (row, major)
fprintf(1,'Reading data\n');
cameras = dlmread(findExampleDataFile('VO_camera_poses_large.txt'));
for i=1:size(cameras,1)
    pose = Pose3(reshape(cameras(i,2:17),4,4)');
    initial.insert(symbol('x',cameras(i,1)),pose);
end

%% load stereo measurements and initialize landmarks
% camera_id landmark_id uL uR v X Y Z
measurements = dlmread(findExampleDataFile('VO_stereo_factors_large.txt'));

fprintf(1,'Creating Graph\n'); tic
for i=1:size(measurements,1)
    sf = measurements(i,:);
    graph.add(GenericStereoFactor3D(StereoPoint2(sf(3),sf(4),sf(5)), stereo_model, ...
        symbol('x', sf(1)), symbol('l', sf(2)), K));
    
    if ~initial.exists(symbol('l',sf(2)))
        % 3D landmarks are stored in camera coordinates: transform
        % to world coordinates using the respective initial pose
        pose = initial.atPose3(symbol('x', sf(1)));
        world_point = pose.transformFrom(Point3(sf(6),sf(7),sf(8)));
        initial.insert(symbol('l',sf(2)), world_point);
    end
end
toc

%% add a constraint on the starting pose
key = symbol('x',1);
first_pose = initial.atPose3(key);
graph.add(NonlinearEqualityPose3(key, first_pose));

%% optimize
fprintf(1,'Optimizing\n'); tic
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely();
toc

%% visualize initial trajectory, final trajectory, and final points
cla; hold on;

plot3DTrajectory(initial, 'r', 1, 0.5);
plot3DTrajectory(result, 'g', 1, 0.5);
plot3DPoints(result);

axis([-5 20 -20 20 0 100]);
axis equal
view(3)
camup([0;1;0]);
