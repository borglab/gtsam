import gtsam.*;

deltaT = 0.01;
timeElapsed = 1000;

times = 0:deltaT:timeElapsed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant global velocity w/ lever arm
disp('--------------------------------------------------------');
disp('Constant global velocity w/ lever arm');
omega = [0;0;0.1];
velocity = [1;0;0];

% Initial state
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;

% Positions
positions = zeros(3, length(times)+1);

i = 2;
for t = times
    velocity1body = currentPoseGlobal.rotation.unrotate(Point3(currentVelocityGlobal));
    R = Rot3.Expmap(omega * deltaT);
    velocity2body = currentPoseGlobal.rotation.compose(R).unrotate(Point3(currentVelocityGlobal));
    [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory(currentPoseGlobal, omega, velocity1body, velocity2body, deltaT);
    
    positions(:,i) = currentPoseGlobal.translation;
    i = i + 1;
end

figure;
plot(positions(1,:), positions(2,:), '.-');
