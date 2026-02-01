clc
clear all
close all

import gtsam.*;

addpath(genpath('./Libraries'))

deltaT = 0.01;
timeElapsed = 25;

times = 0:deltaT:timeElapsed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant body velocity w/ lever arm
disp('--------------------------------------------------------');
disp('Constant body velocity w/ lever arm');
omega = [0;0;0.1];
velocity = [1;0;0];
RIMUinBody = Rot3.Rz(-pi/2);
% RIMUinBody = eye(3)
IMUinBody = Pose3(RIMUinBody, Point3([1;0;0]));

% Initial state (body)
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;
% Initial state (IMU)
currentPoseGlobalIMU = Pose3; %currentPoseGlobal.compose(IMUinBody);
%currentVelocityGlobalIMU = IMUinBody.rotation.unrotate(Point3(velocity)); % no Coriolis here?
currentVelocityGlobalIMU = IMUinBody.rotation.unrotate( ...
     Point3(velocity + cross(omega, IMUinBody.translation)));
   

% Positions
% body trajectory
positions = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation;
poses(1).p = positions(:,1);
poses(1).R = currentPoseGlobal.rotation.matrix;

% Expected IMU trajectory (from body trajectory and lever arm)
positionsIMUe = zeros(3, length(times)+1);
positionsIMUe(:,1) = IMUinBody.compose(currentPoseGlobalIMU).translation;
posesIMUe(1).p = positionsIMUe(:,1);
posesIMUe(1).R = poses(1).R * IMUinBody.rotation.matrix;

% Integrated IMU trajectory (from IMU measurements)
positionsIMU = zeros(3, length(times)+1);
positionsIMU(:,1) = IMUinBody.compose(currentPoseGlobalIMU).translation;
posesIMU(1).p = positionsIMU(:,1);
posesIMU(1).R = IMUinBody.compose(currentPoseGlobalIMU).rotation.matrix;

i = 2;
for t = times
    [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
        currentPoseGlobal, omega, velocity, velocity, deltaT);
    
    acc_omega = imuSimulator.calculateIMUMeasurement( ...
        omega, omega, velocity, velocity, deltaT, IMUinBody);
    
    [ currentPoseGlobalIMU, currentVelocityGlobalIMU ] = imuSimulator.integrateIMUTrajectory( ...
        currentPoseGlobalIMU, currentVelocityGlobalIMU, acc_omega, deltaT);

    % Store data in some structure for statistics and plots  
    positions(:,i) = currentPoseGlobal.translation;
    positionsIMUe(:,i) = currentPoseGlobal.translation + currentPoseGlobal.rotation.matrix * IMUinBody.translation;
    positionsIMU(:,i) = IMUinBody.compose(currentPoseGlobalIMU).translation;
    
    poses(i).p = positions(:,i);
    posesIMUe(i).p = positionsIMUe(:,i);
    posesIMU(i).p = positionsIMU(:,i);   
    
    poses(i).R = currentPoseGlobal.rotation.matrix;
    posesIMUe(i).R = poses(i).R * IMUinBody.rotation.matrix;
    posesIMU(i).R = IMUinBody.compose(currentPoseGlobalIMU).rotation.matrix;      
    i = i + 1;   
end


figure(1)
plot_trajectory(poses, 50, '-k', 'body trajectory',0.1,0.75,1)
hold on
plot_trajectory(posesIMU, 50, '-r', 'imu trajectory',0.1,0.75,1)

figure(2)
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot(positionsIMU(1,:), positionsIMU(2,:), '-r');
plot(positionsIMUe(1,:), positionsIMUe(2,:), ':k');
axis equal;

disp('Mismatch between final integrated IMU position and expected IMU position')
disp(norm(posesIMUe(end).p - posesIMU(end).p))
disp('Mismatch between final integrated IMU orientation and expected IMU orientation')
disp(norm(posesIMUe(end).R - posesIMU(end).R))

