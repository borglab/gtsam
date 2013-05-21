import gtsam.*;

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
positions = zeros(3, length(times)+1);
positionsIMU = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation.vector;
positionsIMU(:,1) = currentPoseGlobalIMU.translation.vector;

i = 2;
for t = times
    [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
        currentPoseGlobal, omega, velocity, velocity, deltaT);
    
    acc_omega = imuSimulator.calculateIMUMeasurement( ...
        omega, omega, velocity, velocity, deltaT, IMUinBody);
    
    [ currentPoseGlobalIMU, currentVelocityGlobalIMU ] = imuSimulator.integrateIMUTrajectory( ...
        currentPoseGlobalIMU, currentVelocityGlobalIMU, acc_omega, deltaT);

    positions(:,i) = currentPoseGlobal.translation.vector;
    positionsIMU(:,i) = currentPoseGlobalIMU.translation.vector;
    i = i + 1;
end

figure;
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot(positionsIMU(1,:), positionsIMU(2,:), '-r');
axis equal;

