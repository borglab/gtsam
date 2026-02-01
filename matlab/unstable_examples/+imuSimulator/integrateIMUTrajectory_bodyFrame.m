function [ finalPose, finalVelocityGlobal ] = integrateIMUTrajectory_bodyFrame( ...
    initialPoseGlobal, initialVelocityGlobal, acc_omegaIMU, deltaT, velocity1Body)

% Before integrating in the body frame we need to compensate for the Coriolis 
% effect
acc_body =  acc_omegaIMU(1:3) - Point3(cross(acc_omegaIMU(4:6), velocity1Body));
% after compensating for coriolis this will be essentially zero
% since we are moving at constant body velocity 

import gtsam.*;
%% Integrate in the body frame
% Integrate rotations
imu2in1 = Rot3.Expmap(acc_omegaIMU(4:6) * deltaT);
% Integrate positions
finalPositionBody = velocity1Body * deltaT + 0.5 * acc_body * deltaT * deltaT;
finalVelocityBody = velocity1Body + acc_body * deltaT; 

%% Express the integrated quantities in the global frame
finalVelocityGlobal = initialVelocityGlobal + (initialPoseGlobal.rotation().rotate(Point3(finalVelocityBody)) );
finalPosition = initialPoseGlobal.translation() + initialPoseGlobal.rotation().rotate( Point3(finalPositionBody)) ;
finalRotation = initialPoseGlobal.rotation.compose(imu2in1);
% Include position and rotation in a pose
finalPose = Pose3(finalRotation, Point3(finalPosition) );

end

