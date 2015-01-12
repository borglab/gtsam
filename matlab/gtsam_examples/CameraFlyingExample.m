clear all;
clc;
import gtsam.*

%% generate a set of cylinders and Samples
fieldSize = Point2([100, 100]');
cylinder_num = 10;
cylinders = cell(cylinder_num, 1);

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
    camera = SimpleCamera.Lookat(trans, cylinders{round(cylinderNum*rand)}.centroid, ...
        Point3([0,0,1]'), K);
    
    incT = Point3(5*rand, 5*rand, 5*rand);
    trans = trans.compose(incT);    
end

%% visibility validation
visiblePoints3 = cylinderSampleProjection(camera, imageSize, cylinders);

%% plot all the projected points
%plotProjectedCylinderSamples(visiblePoints3, cameraPoses{1}, figID);

%% setp up monocular camera and get measurements
pts2dTracksMono = points2DTrackMonocular(K, cameraPoses, imageSize, cylinders);

%% set up stereo camera and get measurements
%pts2dTracksStereo = points2DTrackStereo(K, cameraPoses, imageSize, cylinders);


% ToDo: plot the trajectories
%plot3DTrajectory();



 


