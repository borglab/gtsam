function [ finalPose, finalVelocityGlobal ] = integrateTrajectory( ...
    initialPose, omega1Body, velocity1Body, velocity2Body, deltaT)
%INTEGRATETRAJECTORY Integrate one trajectory step

import gtsam.*;
% Rotation: R^1_2
body2in1 = Rot3.Expmap(omega1Body * deltaT);
% Velocity 2 in frame 1: v^1_2 = R^1_2 v^2_2
velocity2inertial = body2in1.rotate(Point3(velocity2Body));
% Acceleration: a^1 = (v^1_2 - v^1_1)/dt
accelBody1 = (velocity2inertial - velocity1Body) / deltaT;

% Velocity 1 in frame W: v^W_1 = R^W_1 v^1_1
initialVelocityGlobal = initialPose.rotation().rotate(Point3(velocity1Body));
% Acceleration in frame W: a^W = R^W_1 a^1
accelGlobal = initialPose.rotation().rotate(Point3(accelBody1));

finalPosition = Point3(initialPose.translation + initialVelocityGlobal * deltaT + 0.5 * accelGlobal * deltaT * deltaT);
finalVelocityGlobal = initialVelocityGlobal + accelGlobal * deltaT;
finalRotation = initialPose.rotation.compose(body2in1);
finalPose = Pose3(finalRotation, finalPosition);

end

