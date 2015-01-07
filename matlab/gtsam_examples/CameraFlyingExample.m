clear all;
clc;
import gtsam.*

cylinder_num = 10;
cylinders = cell(cylinder_num, 1);

% generate a set of cylinders
for i = 1:cylinder_num
    cylinder_center = Point2([10, 5 * i]');
    cylinders{i,1} = cylinderSampling(cylinder_center, 1, 5, 30);
end

% visibility validation
%camera_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),y_shift);

K = Cal3_S2(525,525,0,320,240);
cam_pose = Pose3();
cam = SimpleCamera(cam_pose, K);

% the projections of all visible 3D points
visiblePoints3 = cylinderSampleProjection(cam, cam_pose, cylinders);

% 

% 


