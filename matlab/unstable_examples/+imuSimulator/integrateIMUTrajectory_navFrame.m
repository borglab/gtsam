function [ finalPose, finalVelocityGlobal ] = integrateIMUTrajectory_navFrame( ...
    initialPoseGlobal, initialVelocityGlobal, acc_omegaIMU, deltaT)
%INTEGRATETRAJECTORY Integrate one trajectory step from IMU measurement

import gtsam.*;
% Integrate rotations
imu2in1 = Rot3.Expmap(acc_omegaIMU(4:6) * deltaT);
finalRotation = initialPoseGlobal.rotation.compose(imu2in1);

intermediateRotation = initialPoseGlobal.rotation.compose( Rot3.Expmap(acc_omegaIMU(4:6) * deltaT/2 ));
% Integrate positions (equation (1) in Lupton)
accelGlobal = intermediateRotation.rotate(Point3(acc_omegaIMU(1:3)));
finalPosition = Point3(initialPoseGlobal.translation ...
    + initialVelocityGlobal * deltaT + 0.5 * accelGlobal * deltaT * deltaT);
finalVelocityGlobal = initialVelocityGlobal + accelGlobal * deltaT;

% Include position and rotation in a pose
finalPose = Pose3(finalRotation, finalPosition);

end

