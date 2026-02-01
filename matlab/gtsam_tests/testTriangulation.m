%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Test triangulation
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Some common constants
sharedCal = Cal3_S2(1500, 1200, 0, 640, 480);

%% Looking along X-axis, 1 meter above ground plane (x-y)
upright = Rot3.Ypr(-pi / 2, 0., -pi / 2);
pose1 = Pose3(upright, Point3(0, 0, 1));
camera1 = PinholeCameraCal3_S2(pose1, sharedCal);

%% create second camera 1 meter to the right of first camera
pose2 = pose1.compose(Pose3(Rot3(), Point3(1, 0, 0)));
camera2 = PinholeCameraCal3_S2(pose2, sharedCal);

%% landmark ~5 meters infront of camera
landmark =Point3 (5, 0.5, 1.2);

%% 1. Project two landmarks into two cameras and triangulate
z1 = camera1.project(landmark);
z2 = camera2.project(landmark);

%% twoPoses
poses = Pose3Vector;
measurements = Point2Vector;

poses.push_back(pose1);
poses.push_back(pose2);
measurements.push_back(z1);
measurements.push_back(z2);

optimize = true;
rank_tol = 1e-9;

triangulated_landmark = triangulatePoint3(poses,sharedCal, measurements, rank_tol, optimize);
CHECK('triangulated_landmark', abs(landmark - triangulated_landmark) < 1e-9);

%% 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
measurements = Point2Vector;
measurements.push_back(z1 + [0.1;0.5]);
measurements.push_back(z2 + [-0.2;0.3]);

triangulated_landmark = triangulatePoint3(poses,sharedCal, measurements, rank_tol, optimize);
CHECK('triangulated_landmark', abs(landmark - triangulated_landmark) < 1e-2);

%% two Poses with Bundler Calibration
bundlerCal = Cal3Bundler(1500, 0, 0, 640, 480);
camera1 = PinholeCameraCal3Bundler(pose1, bundlerCal);
camera2 = PinholeCameraCal3Bundler(pose2, bundlerCal);

z1 = camera1.project(landmark);
z2 = camera2.project(landmark);

measurements = Point2Vector;
measurements.push_back(z1);
measurements.push_back(z2);

triangulated_landmark = triangulatePoint3(poses,bundlerCal, measurements, rank_tol, optimize);
CHECK('triangulated_landmark', abs(landmark - triangulated_landmark) < 1e-9);
