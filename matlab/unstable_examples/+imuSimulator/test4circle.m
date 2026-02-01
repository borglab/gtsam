clc
clear all
close all

import gtsam.*;

deltaT = 0.001;
timeElapsed = 1;
times = 0:deltaT:timeElapsed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--------------------------------------------------------');
disp('Integration in body frame VS integration in navigation frame');
disp('TOY EXAMPLE:');
disp('- Body moves along a circular trajectory with constant rotation rate -omega- and constant -velocity- (in the body frame)');
disp('- We compare two integration schemes: integating in the navigation frame (similar to Lupton approach) VS integrating in the body frame')
disp('- Navigation frame is assumed inertial for simplicity')
omega = [0;0;2*pi];
velocity = [1;0;0];

%% Set initial conditions for the true trajectory and for the estimates
% (one estimate is obtained by integrating in the body frame, the other
% by integrating in the navigation frame)
% Initial state (body)
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;
% Initial state estimate (integrating in navigation frame)
currentPoseGlobalIMUnav = currentPoseGlobal;
currentVelocityGlobalIMUnav = currentVelocityGlobal;
% Initial state estimate (integrating in the body frame)
currentPoseGlobalIMUbody = currentPoseGlobal;
currentVelocityGlobalIMUbody = currentVelocityGlobal;

%% Prepare data structures for actual trajectory and estimates
% Actual trajectory
positions = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation;
poses(1).p = positions(:,1);
poses(1).R = currentPoseGlobal.rotation.matrix;

% Trajectory estimate (integrated in the navigation frame)
positionsIMUnav = zeros(3, length(times)+1);
positionsIMUnav(:,1) = currentPoseGlobalIMUbody.translation;
posesIMUnav(1).p = positionsIMUnav(:,1);
posesIMUnav(1).R = poses(1).R;

% Trajectory estimate (integrated in the body frame)
positionsIMUbody = zeros(3, length(times)+1);
positionsIMUbody(:,1) = currentPoseGlobalIMUbody.translation;
posesIMUbody(1).p = positionsIMUbody(:,1);
posesIMUbody(1).R = poses(1).R;

%% Main loop
i = 2;
for t = times
  %% Create the actual trajectory, using the velocities and
  % accelerations in the inertial frame to compute the positions
  [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
    currentPoseGlobal, omega, velocity, velocity, deltaT);
  
  %% Simulate IMU measurements, considering Coriolis effect 
  % (in this simple example we neglect gravity and there are no other forces acting on the body)
  acc_omega = imuSimulator.calculateIMUMeas_coriolis( ...
    omega, omega, velocity, velocity, deltaT);
  
  %% Integrate in the body frame
  [ currentPoseGlobalIMUbody, currentVelocityGlobalIMUbody ] = imuSimulator.integrateIMUTrajectory_bodyFrame( ...
    currentPoseGlobalIMUbody, currentVelocityGlobalIMUbody, acc_omega, deltaT, velocity);

  %% Integrate in the navigation frame
  [ currentPoseGlobalIMUnav, currentVelocityGlobalIMUnav ] = imuSimulator.integrateIMUTrajectory_navFrame( ...
    currentPoseGlobalIMUnav, currentVelocityGlobalIMUnav, acc_omega, deltaT);
  
  %% Store data in some structure for statistics and plots
  positions(:,i) = currentPoseGlobal.translation;
  positionsIMUbody(:,i) = currentPoseGlobalIMUbody.translation;
  positionsIMUnav(:,i) = currentPoseGlobalIMUnav.translation;
  % - 
  poses(i).p = positions(:,i);
  posesIMUbody(i).p = positionsIMUbody(:,i);
  posesIMUnav(i).p = positionsIMUnav(:,i); 
  % -
  poses(i).R = currentPoseGlobal.rotation.matrix;
  posesIMUbody(i).R = currentPoseGlobalIMUbody.rotation.matrix;
  posesIMUnav(i).R = currentPoseGlobalIMUnav.rotation.matrix;
  i = i + 1;
end

figure(1)
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot(positionsIMUbody(1,:), positionsIMUbody(2,:), '-r');
plot(positionsIMUnav(1,:), positionsIMUnav(2,:), ':k');
axis equal;
legend('true trajectory', 'traj integrated in body', 'traj integrated in nav')


