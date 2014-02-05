%% coriolisExample.m
% Author(s): David Jensen (david.jensen@gtri.gatech.edu)
% This script demonstrates the relationship between object motion in inertial and rotating reference frames.
% For this example, we consider a fixed (inertial) reference frame located at the center of the earth and initially 
% coincident with the rotating ECEF reference frame (X towards 0 longitude, Y towards 90 longitude, Z towards 90 latitude),
% which rotates with the earth.
% A body is moving in the positive Z-direction and positive Y-direction with respect to the fixed reference frame.
% Its position is plotted in both the fixed and rotating reference frames to simulate how an observer in each frame would
% experience the body's motion.

clc
clear all
close all

import gtsam.*;

%% General configuration
deltaT = 0.1;
timeElapsed = 10;
times = 0:deltaT:timeElapsed;

%omega = [0;0;7.292115e-5]; % Earth Rotation
omega = [0;0;5*pi/10];
velocity = [0;0;0];                 % initially not moving
accelFixed = [0;0.1;0.1];           % accelerate in the positive z-direction
initialPosition = [0; 1.05; 0];     % start along the positive x-axis

%% Initial state of the body in the fixed in rotating frames should be the same
currentPoseFixedGT = Pose3(Rot3, Point3(initialPosition));
currentVelocityFixedGT = velocity;

currentPoseRotatingGT = currentPoseFixedGT;
currentPoseRotatingFrame = Pose3;

%% Initialize storage variables
positionsFixedGT = zeros(3, length(times)+1);
positionsRotatingGT = zeros(3, length(times)+1);

positionsFixedGT(:,1) = currentPoseFixedGT.translation.vector;
positionsRotatingGT(:,1) = currentPoseRotatingGT.translation.vector;

changePoseRotatingFrame = Pose3.Expmap([omega*deltaT; 0; 0; 0]);

poses(1).p = currentPoseRotatingFrame.translation.vector;
poses(1).R = currentPoseRotatingFrame.rotation.matrix;

h = figure(1);

%% Main loop: iterate through 
i = 2;
for t = times
    %% Create ground truth trajectory
    % Update the position and velocity
    % x_t = x_0 + v_0*dt + 1/2*a_0*dt^2
    % v_t = v_0 + a_0*dt
    currentPositionFixedGT = Point3(currentPoseFixedGT.translation.vector ...
        + currentVelocityFixedGT * deltaT + 0.5 * accelFixed * deltaT * deltaT);
    currentVelocityFixedGT = currentVelocityFixedGT + accelFixed * deltaT;
    
    currentPoseFixedGT = Pose3(Rot3, currentPositionFixedGT);
    
    % Rotate pose in fixed frame to get pose in rotating frame
    currentPoseRotatingFrame = currentPoseRotatingFrame.compose(changePoseRotatingFrame);
    currentPoseRotatingGT = currentPoseFixedGT.transform_to(currentPoseRotatingFrame);
    
    % Store GT (ground truth) poses
    positionsFixedGT(:,i) = currentPoseFixedGT.translation.vector;
    positionsRotatingGT(:,i) = currentPoseRotatingGT.translation.vector;
    poses(i).p = currentPoseRotatingFrame.translation.vector;
    poses(i).R = currentPoseRotatingFrame.rotation.matrix;
    
    % incremental plotting for animation
    figure(h)
    plot_trajectory(poses(i),1, '-k', 'Rotating Frame',0.1,0.75,1)
    hold on;
    plot3(positionsFixedGT(1,:), positionsFixedGT(2,:), positionsFixedGT(3,:));
    plot3(positionsRotatingGT(1,:), positionsRotatingGT(2,:), positionsRotatingGT(3,:), '-r');
    plot3(positionsFixedGT(1,1), positionsFixedGT(2,1), positionsFixedGT(3,1), 'o');
    plot3(positionsFixedGT(1,end), positionsFixedGT(2,end), positionsFixedGT(3,end), 'x');
    plot3(positionsRotatingGT(1,1), positionsRotatingGT(2,1), positionsRotatingGT(3,1), 'or');
    plot3(positionsRotatingGT(1,end), positionsRotatingGT(2,end), positionsRotatingGT(3,end), 'xr');
    hold off;
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    axis equal
    grid on;
    pause(0.1);
    
    i = i + 1;
end

%% Final plotting
figure
sphere  % sphere for reference
hold on;
% trajectories in Fixed and Rotating frames
plot3(positionsFixedGT(1,:), positionsFixedGT(2,:), positionsFixedGT(3,:));
plot3(positionsRotatingGT(1,:), positionsRotatingGT(2,:), positionsRotatingGT(3,:), '-r');

% beginning and end points of Fixed
plot3(positionsFixedGT(1,1), positionsFixedGT(2,1), positionsFixedGT(3,1), 'o');
plot3(positionsFixedGT(1,end), positionsFixedGT(2,end), positionsFixedGT(3,end), 'x');

% beginning and end points of Rotating
plot3(positionsRotatingGT(1,1), positionsRotatingGT(2,1), positionsRotatingGT(3,1), 'or');
plot3(positionsRotatingGT(1,end), positionsRotatingGT(2,end), positionsRotatingGT(3,end), 'xr');

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
grid on;
hold off;
