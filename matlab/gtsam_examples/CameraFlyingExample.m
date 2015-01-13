clear all;
clc;
clf;
import gtsam.*

%% define the options 
options.fieldSize = Point2([100, 100]');
options.cylinderNum = 10;
options.poseNum = 20;
options.monoK = Cal3_S2(525,525,0,320,240);
options.stereoK = Cal3_S2Stereo(721,721,0.0,609,172,0.53715); % read from the VO calibration file
options.imageSize = Point2([640, 480]');

%% generate a set of cylinders and Samples
cylinderNum = options.cylinderNum;
cylinders = cell(cylinderNum, 1);

% It seems random generated cylinders doesn't work that well
% Now it set up a circle of cylinders
theta = 0;
for i = 1:cylinderNum
    theta = theta + 2*pi / 10;
    x = 10 * cos(theta) + options.fieldSize.x/2;
    y = 10 * sin(theta) + options.fieldSize.y/2;
    baseCentroid = Point2([x, y]');
    cylinders{i,1} = cylinderSampling(baseCentroid, 1, 5, 1);
end

%% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
figID = 1;
figure(figID);
plotCylinderSamples(cylinders, options.fieldSize, figID);

%% generate camera trajectories: a circle 
KMono = Cal3_S2(525,525,0,320,240);
imageSize = Point2([640, 480]');
cameraPoses = cell(options.poseNum, 1);
% Generate ground truth trajectory r.w.t. the field center
theta = 0;
r = 40;
for i = 1:options.poseNum    
    theta = (i-1)*2*pi/options.poseNum;
    t = Point3([r*cos(theta) + options.fieldSize.x/2, ...
        r*sin(theta) + options.fieldSize.y/2, 10]');
    camera = SimpleCamera.Lookat(t, ... 
        Point3(options.fieldSize.x/2, options.fieldSize.y/2, 0), ...
        Point3([0,0,1]'), KMono);    
    cameraPoses{i} = camera.pose;
end

%% visibility validation
% for a simple test, it will be removed later
%visiblePoints3 = cylinderSampleProjection(cameras{1}, imageSize, cylinders);

%% plot all the projected points
%plotProjectedCylinderSamples(visiblePoints3, cameraPoses{1}, figID);

%% setp up monocular camera and get measurements
pts2dTracksMono = points2DTrackMonocular(KMono, cameraPoses, imageSize, cylinders);

%% set up stereo camera and get measurements
% load stereo calibration
calib = dlmread(findExampleDataFile('VO_calibration.txt'));
KStereo = Cal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
camerasStereo = cell(options.poseNum, 1);

%pts2dTracksStereo = points2DTrackStereo(KStereo, cameraPoses, imageSize, cylinders);

% plot the 2D tracks

% ToDo: plot the trajectories
%plot3DTrajectory();



 


