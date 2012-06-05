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

%% Load calibration
% format: fx fy skew cx cy baseline
calib = dlmread('../Data/VO_calibration.txt');
K = gtsamCal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
stereo_model = gtsamSharedNoiseModel_Sigmas([1.0; 1.0; 1.0]);

%% create empty graph and values
graph = visualSLAMGraph;
initial = visualSLAMValues;


%% load the initial poses from VO
% row format: camera_id 4x4 pose (row, major)
c = dlmread('../Data/VO_camera_poses.txt');

for i=1:size(c,1)
    pose = gtsamPose3(reshape(c(i,2:17),4,4)');
    initial.insertPose(symbol('x',c(i,1)),pose);
end

%% load stereo measurements and initialize landmarks
% camera_id landmark_id uL uR v X Y Z
m = dlmread('../Data/VO_stereo_factors.txt');

for i=1:size(m,1)
    sf = m(i,:);
    graph.addStereoMeasurement(gtsamStereoPoint2(sf(3),sf(4),sf(5)), stereo_model, ...
    symbol('x', sf(1)), symbol('l', sf(2)), K);

    if ~initial.exists(symbol('l',sf(2)))
        % 3D landmarks are stored in camera coordinates: transform 
        % to world coordinates using the respective initial pose
        pose = initial.pose(symbol('x', sf(1)));
        world_point = pose.transform_from(gtsamPoint3(sf(6),sf(7),sf(8)));
        initial.insertPoint(symbol('l',sf(2)), world_point);
    end
end

%% add a constraint on the starting pose
key = symbol('x',1);
first_pose = initial.pose(key);
graph.addPoseConstraint(symbol('x',1), first_pose);

%% optimize
result = graph.optimize(initial);

%% visualize initial trajectory, final trajectory, and final points
figure(1); clf;

% initial trajectory in red
plot3(initial.xs(),initial.ys(),initial.zs(), '-*r','LineWidth',2);
hold on;

% final trajectory in green
plot3(result.xs(),result.ys(),result.zs(), '-*g','LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% switch to XZ view
view([0 0]);

% optimized 3D points
points = result.points();
plot3(points(:,1),points(:,2),points(:,3),'.');

axis([-30 30 -30 30 0 60]);
