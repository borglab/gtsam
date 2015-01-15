%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief A camera flying example through a field of cylinder landmarks
% @author Zhaoyang Lv
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;
clf;
import gtsam.*

%% define the options 
% the testing field size
options.fieldSize = Point2([100, 100]');
% the number of cylinders
options.cylinderNum = 20;
% point density on cylinder
options.density = 1;

% The number of camera poses
options.poseNum = 20;
% covariance scaling factor
options.scale = 1;


%% Camera Setup
% Monocular Camera Calibration
options.monoK = Cal3_S2(525,525,0,320,240);
% Stereo Camera Calibration
options.stereoK = Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2);
% the image size of camera
options.imageSize = Point2([640, 480]');
% use Monocular camera or Stereo camera
options.Mono = false;
% fps for image
options.fps = 20;
% camera flying speed
options.speed = 20;


%% test1: visibility test in monocular camera 
cylinders{1}.centroid = Point3(30, 50, 5);
cylinders{2}.centroid = Point3(50, 50, 5);
cylinders{3}.centroid = Point3(70, 50, 5);

for i = 1:3
    cylinders{i}.radius = 5;
    cylinders{i}.height = 10;
    
    cylinders{i}.Points{1} = cylinders{i}.centroid.compose(Point3(-cylinders{i}.radius, 0, 0));
    cylinders{i}.Points{2} = cylinders{i}.centroid.compose(Point3(cylinders{i}.radius, 0, 0));
end

camera = SimpleCamera.Lookat(Point3(10, 50, 10), ... 
    Point3(options.fieldSize.x/2, options.fieldSize.y/2, 0), ...
    Point3([0,0,1]'), options.monoK); 

pose = camera.pose;
prjMonoResult = cylinderSampleProjection(options.monoK, pose, options.imageSize, cylinders);

%% test2: visibility test in stereo camera  
prjStereoResult = cylinderSampleProjectionStereo(options.stereoK, pose, options.imageSize, cylinders);

%% generate a set of cylinders and samples
cylinderNum = options.cylinderNum;
cylinders = cell(cylinderNum, 1);

% It seems random generated cylinders doesn't work that well
% Now it set up a circle of cylinders
theta = 0;
for i = 1:cylinderNum
    theta = theta + 2*pi/10;
    x = 30 * rand * cos(theta) + options.fieldSize.x/2;
    y = 20 * rand * sin(theta) + options.fieldSize.y/2;
    baseCentroid = Point2([x, y]');
    cylinders{i,1} = cylinderSampling(baseCentroid, 1, 5, options.density);
end


%% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
figID = 1;
figure(figID);
plotCylinderSamples(cylinders, options.fieldSize, figID);

% %% generate ground truth camera trajectories: a circle 
% KMono = Cal3_S2(525,525,0,320,240);
% cameraPoses = cell(options.poseNum, 1);
% theta = 0;
% r = 40;
% for i = 1:options.poseNum    
%     theta = (i-1)*2*pi/options.poseNum;
%     t = Point3([r*cos(theta) + options.fieldSize.x/2, ...
%         r*sin(theta) + options.fieldSize.y/2, 10]');
%     camera = SimpleCamera.Lookat(t, ... 
%         Point3(options.fieldSize.x/2, options.fieldSize.y/2, 0), ...
%         Point3([0,0,1]'), options.monoK);    
%     cameraPoses{i} = camera.pose;
% end

%% generate ground truth camera trajectories: a line
KMono = Cal3_S2(525,525,0,320,240);
cameraPoses = cell(options.poseNum, 1);
theta = 0;
for i = 1:options.poseNum    
    t = Point3([(i-1)*(options.fieldSize.x - 20)/options.poseNum + 20, ...
        15, 10]');
    camera = SimpleCamera.Lookat(t, ... 
        Point3(options.fieldSize.x/2, options.fieldSize.y/2, 0), ...
        Point3([0,0,1]'), options.monoK);    
    cameraPoses{i} = camera.pose;
end


%% set up camera and get measurements
if options.Mono 
    % use Monocular Camera
    pts2dTracksMono = points2DTrackMonocular(options.monoK, cameraPoses, ...
        options.imageSize, cylinders);
else 
    % use Stereo Camera
    pts2dTracksStereo = points2DTrackStereo(options.stereoK, cameraPoses, ...
        options.imageSize, cylinders);
    
    figID = 2;
    plotProjectedCylinderSamples(pts2dTracksStereo.pt3d, pts2dTracksStereo.cov, figID);

end

%% plot all the projected points


% plot the 2D tracks

% ToDo: plot the trajectories
%plot3DTrajectory();



 


