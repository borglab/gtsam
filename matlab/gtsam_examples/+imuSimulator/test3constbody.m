import gtsam.*;

close all
clc

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
IMUinBody = Pose3(Rot3, Point3([0;1;0]));

% Initial state
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;
currentPoseGlobalIMU = currentPoseGlobal.compose(IMUinBody);
currentVelocityGlobalIMU = IMUinBody.rotation.unrotate(Point3(velocity)).vector;

% Positions
% body trajectory
positions = zeros(3, length(times)+1);
% Expected IMU trajectory (from body trajectory and lever arm)
positionsIMUe = zeros(3, length(times)+1);
% Integrated IMU trajectory (from IMU measurements)
positionsIMU = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation.vector;
positionsIMUe(:,1) = currentPoseGlobalIMU.translation.vector;
positionsIMU(:,1) = currentPoseGlobalIMU.translation.vector;

poses(1).p = positions(:,1);
posesIMUe(1).p = positionsIMUe(:,1);
posesIMU(1).p = positionsIMU(:,1);

poses(1).R = currentPoseGlobal.rotation.matrix;
posesIMUe(1).R = poses(1).R * IMUinBody.rotation.matrix;
posesIMU(1).R = currentPoseGlobalIMU.rotation.matrix;

i = 2;
for t = times
    [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
        currentPoseGlobal, omega, velocity, velocity, deltaT);
    
    acc_omega = imuSimulator.calculateIMUMeasurement( ...
        omega, omega, velocity, velocity, deltaT, IMUinBody);
    
    [ currentPoseGlobalIMU, currentVelocityGlobalIMU ] = imuSimulator.integrateIMUTrajectory( ...
        currentPoseGlobalIMU, currentVelocityGlobalIMU, acc_omega, deltaT);

    positions(:,i) = currentPoseGlobal.translation.vector;
    positionsIMUe(:,i) = currentPoseGlobal.translation.vector + currentPoseGlobal.rotation.matrix * IMUinBody.translation.vector;
    positionsIMU(:,i) = currentPoseGlobalIMU.translation.vector;
    
    poses(i).p = positions(:,i);
    posesIMUe(i).p = positionsIMUe(:,i);
    posesIMU(i).p = positionsIMU(:,i);   
    
    poses(i).R = currentPoseGlobal.rotation.matrix;
    posesIMUe(i).R = poses(i).R * IMUinBody.rotation.matrix;
    posesIMU(i).R = currentPoseGlobalIMU.rotation.matrix;      
    i = i + 1;   
end


figure
plot_trajectory(poses, 50, '-k', 'body trajectory',0.1,0.75,1)

figure;
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot(positionsIMU(1,:), positionsIMU(2,:), '-r');
plot(positionsIMUe(1,:), positionsIMUe(2,:), ':k');
axis equal;

