clear all;
clc;
import gtsam.*

cylinder_num = 20;
cylinders = cell(cylinder_num, 1);

% generate a set of cylinders
fieldSize = Point2([100, 100]');

% random generate cylinders on the fields
for i = 1:cylinder_num
    baseCentroid = Point2([fieldSize.x * rand, fieldSize.y * rand]');
    cylinders{i,1} = cylinderSampling(baseCentroid, 1, 5, 30);
end

% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
plotCylinderSamples(cylinders, fieldSize);

% visibility validation

K = Cal3_S2(525,525,0,320,240);
cameraPose = Pose3();             % now set to be default

% the projections of all visible 3D points
% visiblePoints3 = cylinderSampleProjection(K, cameraPose, cylinders);




 


