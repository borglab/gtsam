clear all;
clc;
clf;
import gtsam.*

%% generate a set of cylinders and Samples
fieldSize = Point2([100, 100]');
cylinderNum = 10;
cylinders = cell(cylinderNum, 1);

% ToDo: it seems random generated cylinders doesn't work that well
% use fixed parameters instead
for i = 1:cylinderNum
    baseCentroid = Point2([fieldSize.x * rand, fieldSize.y * rand]');
    cylinders{i,1} = cylinderSampling(baseCentroid, 1, 5, 1);
end

%% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
figID = 1;
figure(figID);
plotCylinderSamples(cylinders, fieldSize, figID);

%% generate camera trajectories
K = Cal3_S2(525,525,0,320,240);
imageSize = Point2([640, 480]');
poseNum = 10;
cameras = cell(poseNum, 1);
trans = Point3();
% To ensure there are landmarks in view, look at one randomly chosen cylinder 
% each time.
for i = 1:poseNum
    cylinderIdx = max(min(round(cylinderNum*rand), 10), 1);
    cameras{i} = SimpleCamera.Lookat(trans, cylinders{cylinderIdx}.centroid, ...
        Point3([0,0,1]'), K);
    
    incT = Point3(5*rand, 5*rand, 5*rand);
    trans = trans.compose(incT);    
end

%% visibility validation
% for a simple test, it will be removed later
visiblePoints3 = cylinderSampleProjection(cameras{1}, imageSize, cylinders);

%% plot all the projected points
%plotProjectedCylinderSamples(visiblePoints3, cameraPoses{1}, figID);

%% setp up monocular camera and get measurements
%pts2dTracksMono = points2DTrackMonocular(cameras, imageSize, cylinders);

%% set up stereo camera and get measurements
% load stereo calibration
calib = dlmread(findExampleDataFile('VO_calibration.txt'));
KStereo = Cal3_S2Stereo(calib(1), calib(2), calib(3), calib(4), calib(5), calib(6));
poseNum = 10;
camerasStereo = cell(poseNum, 1);
trans = Point3();
for i = 1:poseNum
    cylinderIdx = max(min(round(cylinderNum*rand), 10), 1);
    camerasStereo{i} = SimpleCamera.Lookat(trans, cylinders{cylinderIdx}.centroid, ...
        Point3([0,0,1]'), KStereo);
    
    incT = Point3(5*rand, 5*rand, 5*rand);
    trans = trans.compose(incT);    
end

pts2dTracksStereo = points2DTrackStereo(camerasStereo, imageSize, cylinders);

% plot the 2D tracks

% ToDo: plot the trajectories
%plot3DTrajectory();



 


