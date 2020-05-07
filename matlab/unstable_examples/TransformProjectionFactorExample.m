%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
import gtsam.*

%% generate some landmarks
nrPoints = 8;
 landmarks = {Point3([20 15 1]'),...
        Point3([22 7 1]'),...
        Point3([20 20 6]'),...
        Point3([24 19 4]'),...
        Point3([26 17 2]'),...
        Point3([12 15 4]'),...
        Point3([25 11 6]'),...
        Point3([23 10 4]')};

fg = NonlinearFactorGraph;
fg.add(NonlinearEqualityPose3(1, Pose3()));
initial = Values;

%% intial landmarks and camera trajectory shifted in + y-direction
y_shift = Point3(0,1,0);

% insert shifted points
for i=1:nrPoints
   initial.insert(100+i,landmarks{i}.compose(y_shift)); 
end

figure(1);
cla
hold on;
plot3DPoints(initial);

%% Actual camera translation coincides with odometry, but -90deg Z-X rotation
camera_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),y_shift);
actual_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),Point3());
initial.insert(1000,camera_transform);

%% insert poses
initial.insert(1, Pose3());


move_forward = Pose3(Rot3(),Point3(1,0,0));
move_circle = Pose3(Rot3.RzRyRx(0.0,0.0,0.2),Point3(1,0,0));
covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
z_cov = noiseModel.Diagonal.Sigmas([1.0;1.0]);
    
K = Cal3_S2(900,900,0,640,480);
cheirality_exception_count = 0;

for i=1:20
    if i > 1
        if i < 11
            initial.insert(i,initial.atPose3(i-1).compose(move_forward));
            fg.add(BetweenFactorPose3(i-1,i, move_forward, covariance));
        else
            initial.insert(i,initial.atPose3(i-1).compose(move_circle));
            fg.add(BetweenFactorPose3(i-1,i, move_circle, covariance));
        end
        
    end
    
    % generate some camera measurements
    cam_pose = initial.atPose3(i).compose(actual_transform);
    gtsam.plotPose3(cam_pose);
    cam = PinholeCameraCal3_S2(cam_pose,K);
    i
    for j=1:nrPoints
        % All landmarks seen in every frame
        try
        z = cam.project(landmarks{j});
        fg.add(TransformProjectionFactorCal3_S2(z, z_cov, i, 1000, 100+j, K));
        catch
            cheirality_exception_count = cheirality_exception_count + 1;
        end % end try/catch
    end  
    
end

fprintf('Cheirality Exception count: %d\n', cheirality_exception_count);

% plot3DTrajectory(initial, 'g-*');

%% camera plotting
for i=1:20
   gtsam.plotPose3(initial.atPose3(i).compose(camera_transform));
end

xlabel('x (m)');
ylabel('y (m)');

disp('Transform before optimization');
initial.atPose3(1000)

params = LevenbergMarquardtParams;
params.setAbsoluteErrorTol(1e-15);
params.setRelativeErrorTol(1e-15);
params.setVerbosity('ERROR');
params.setVerbosityLM('VERBOSE');

optimizer = LevenbergMarquardtOptimizer(fg, initial, params);
result = optimizer.optimizeSafely();

disp('Transform after optimization');
result.atPose3(1000)       % results is empty here. optimizer doesn't generate result?

axis([0 25 0 25 0 10]);
axis equal
view(-37,40)

