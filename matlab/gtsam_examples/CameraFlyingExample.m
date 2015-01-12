clear all;
clc;
import gtsam.*

cylinder_num = 10;
cylinders = cell(cylinder_num, 1);

% generate a set of cylinders
fieldSize = Point2([100, 100]');

% random generate cylinders on the fields
for i = 1:cylinder_num
    baseCentroid = Point2([fieldSize.x * rand, fieldSize.y * rand]');
    cylinders{i,1} = cylinderSampling(baseCentroid, 1, 5, 1);
end

% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
figID = 1;
figure(figID);
plotCylinderSamples(cylinders, fieldSize, figID);

% visibility validation

% generate camera trajectories
K = Cal3_S2(525,525,0,320,240);
imageSize = Point2([640, 480]');
poseNum = 10;
cameraPoses = cell(poseNum, 1);
cameraPoses{1} = Pose3();
cameras = cell(poseNum, 1);
for i = 2:poseNum
    incRot = Rot3.RzRyRx(0,0,pi/4);
    incT = Point3(5*rand, 5*rand, 5*rand);
    cameraPoses{i} = cameraPoses{i-1}.compose(Pose3(incRot, incT));    
end

[visiblePoints3, ~] = cylinderSampleProjection(K, cameraPoses{1}, imageSize, cylinders);

plotPose3(cameraPoses{1}, 5 )
% plot all the projected points
plotProjectedCylinderSamples(visiblePoints3, cameraPoses{1}, figID);

pts2dTracksMono = points2DTrackMonocular(K, cameraPoses, imageSize, cylinders);

%pts2dTracksStereo = points2DTrackStereo(K, cameraPoses, imageSize, cylinders);


% ToDo: plot the trajectories
%plot3DTrajectory();



 


