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
import gtsam.*
% format: fx fy skew cx cy baseline
calib = dlmread('../../examples/Data/VO_calibration.txt');
K = Cal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
stereo_model = noiseModel.Diagonal.Sigmas([1.0; 1.0; 1.0]);

%% create empty graph and values
graph = visualSLAM.Graph;
initial = visualSLAM.Values;


%% load the initial poses from VO
% row format: camera_id 4x4 pose (row, major)
import gtsam.*
fprintf(1,'Reading data\n');
cameras = dlmread('../../examples/Data/VO_camera_poses_large.txt');
for i=1:size(cameras,1)
    pose = Pose3(reshape(cameras(i,2:17),4,4)');
    initial.insertPose(symbol('x',cameras(i,1)),pose);
end

%% load stereo measurements and initialize landmarks
% camera_id landmark_id uL uR v X Y Z
import gtsam.*
measurements = dlmread('../../examples/Data/VO_stereo_factors_large.txt');

fprintf(1,'Creating Graph\n'); tic
for i=1:size(measurements,1)
    sf = measurements(i,:);
    graph.addStereoMeasurement(StereoPoint2(sf(3),sf(4),sf(5)), stereo_model, ...
        symbol('x', sf(1)), symbol('l', sf(2)), K);
    
    if ~initial.exists(symbol('l',sf(2)))
        % 3D landmarks are stored in camera coordinates: transform
        % to world coordinates using the respective initial pose
        pose = initial.pose(symbol('x', sf(1)));
        world_point = pose.transform_from(Point3(sf(6),sf(7),sf(8)));
        initial.insertPoint(symbol('l',sf(2)), world_point);
    end
end
toc

%% add a constraint on the starting pose
key = symbol('x',1);
first_pose = initial.pose(key);
graph.addPoseConstraint(key, first_pose);

%% optimize
fprintf(1,'Optimizing\n'); tic
result = graph.optimize(initial,1);
toc

%% visualize initial trajectory, final trajectory, and final points
figure(1); clf; hold on;

% initial trajectory in red (rotated so Z is up)
plot3(initial.zs(),-initial.xs(),-initial.ys(), '-*r','LineWidth',2);

% final trajectory in green (rotated so Z is up)
plot3(result.zs(),-result.xs(),-result.ys(), '-*g','LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% switch to XZ view
view([0 0]);

% optimized 3D points (rotated so Z is up)
points = result.points();
plot3(points(:,3),-points(:,1),-points(:,2),'.');

axis([0 100 -20 20 -5 20]);
axis equal
view(3)
