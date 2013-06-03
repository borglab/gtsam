function [ finalPose, finalVelocityGlobal ] = integrateIMUTrajectory_bodyFrame( ...
    initialPoseGlobal, initialVelocityGlobal, acc_omegaIMU, deltaT, velocity1Body)
%INTEGRATETRAJECTORY Integrate one trajectory step from IMU measurement

acc_body = 0 ; % after compensating for coriolis 

import gtsam.*;
% Integrate rotations
imu2in1 = Rot3.Expmap(acc_omegaIMU(4:6) * deltaT);
finalRotation = initialPoseGlobal.rotation.compose(imu2in1);

% Integrate positions
finalPositionBody = velocity1Body * deltaT + 0.5 * acc_body * deltaT * deltaT;
finalVelocityBody = velocity1Body + acc_body * deltaT; 

finalPosition = initialPoseGlobal.translation().vector() + initialPoseGlobal.rotation().rotate( Point3(finalPositionBody)).vector() ;
finalVelocityGlobal = initialVelocityGlobal + (initialPoseGlobal.rotation().rotate(Point3(finalVelocityBody)).vector() );

% Include position and rotation in a pose
finalPose = Pose3(finalRotation, Point3(finalPosition) );

end

