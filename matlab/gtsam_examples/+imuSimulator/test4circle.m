clc
clear all
close all

import gtsam.*;

addpath(genpath('./Libraries'))

deltaT = 0.001;
timeElapsed = 1;
times = 0:deltaT:timeElapsed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant body velocity w/ lever arm
disp('--------------------------------------------------------');
disp('Integration in body frame VS integration in navigation frame');
omega = [0;0;2*pi];
velocity = [1;0;0];
RIMUinBody = eye(3);
IMUinBody = Pose3(Rot3(RIMUinBody), Point3([0;0;0]));

% Initial state (body)
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;
% Initial state (IMU)
currentPoseGlobalIMU = currentPoseGlobal.compose(IMUinBody);
currentPoseGlobalIMUnav = currentPoseGlobalIMU;
currentVelocityGlobalIMU = IMUinBody.rotation.unrotate( ...
     Point3(velocity + cross(omega, IMUinBody.translation.vector))).vector;
currentVelocityGlobalIMUnav =currentVelocityGlobalIMU;
   
% Positions
% body trajectory
positions = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation.vector;
poses(1).p = positions(:,1);
poses(1).R = currentPoseGlobal.rotation.matrix;

% Expected IMU trajectory (from body trajectory and lever arm)
positionsIMUnav = zeros(3, length(times)+1);
positionsIMUnav(:,1) = currentPoseGlobalIMU.translation.vector;
posesIMUnav(1).p = positionsIMUnav(:,1);
posesIMUnav(1).R = poses(1).R;

% Integrated IMU trajectory (from IMU measurements)
positionsIMU = zeros(3, length(times)+1);
positionsIMU(:,1) = currentPoseGlobalIMU.translation.vector;
posesIMU(1).p = positionsIMU(:,1);
posesIMU(1).R = poses(1).R;

i = 2;
for t = times
  % this creates the actual trajectory, using the velocities and
  % accelerations in the inertial frame to compute the positions
    [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
        currentPoseGlobal, omega, velocity, velocity, deltaT);
  
    acc_omega = imuSimulator.calculateIMUMeas_coriolis( ...
        omega, omega, velocity, velocity, deltaT, IMUinBody);
    
    [ currentPoseGlobalIMU, currentVelocityGlobalIMU ] = imuSimulator.integrateIMUTrajectory_bodyFrame( ...
        currentPoseGlobalIMU, currentVelocityGlobalIMU, acc_omega, deltaT, velocity);
          
    [ currentPoseGlobalIMUnav, currentVelocityGlobalIMUnav ] = imuSimulator.integrateIMUTrajectory( ...
        currentPoseGlobalIMUnav, currentVelocityGlobalIMUnav, acc_omega, deltaT);

    % Store data in some structure for statistics and plots  
    positions(:,i) = currentPoseGlobal.translation.vector;
    positionsIMU(:,i) = currentPoseGlobalIMU.translation.vector;
    positionsIMUnav(:,i) = currentPoseGlobalIMUnav.translation.vector;
    
    poses(i).p = positions(:,i);    
    posesIMU(i).p = positionsIMU(:,i);
    posesIMUnav(i).p = positionsIMUnav(:,i);
    
    poses(i).R = currentPoseGlobal.rotation.matrix;
    posesIMU(i).R = currentPoseGlobalIMU.rotation.matrix;  
    posesIMUnav(i).R = currentPoseGlobalIMUnav.rotation.matrix;
    i = i + 1;   
end

stl = 0.1 * [0.1,0.5,1];

figure(1)
plot_trajectory(poses, 10, '-k', 'body trajectory',stl(1),stl(2),stl(2))
hold on
plot_trajectory(posesIMUnav, 10, '-r', 'imu trajectory',stl(1),stl(2),stl(2))

figure(2)
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot(positionsIMU(1,:), positionsIMU(2,:), '-r');
plot(positionsIMUnav(1,:), positionsIMUnav(2,:), ':k');
axis equal;

disp('Mismatch between final integrated IMU position and expected IMU position')
disp(norm(posesIMUnav(end).p - posesIMU(end).p))
disp('Mismatch between final integrated IMU orientation and IMU orientation')
disp(norm(posesIMUnav(end).R - posesIMU(end).R))

