function [ finalPose, finalVelocityGlobal ] = integrateIMUTrajectory( ...
    initialPoseGlobal, initialVelocityGlobal, acc_omegaIMU, deltaT)
%INTEGRATETRAJECTORY Integrate one trajectory step from IMU measurement

import gtsam.*;

imu2in1 = Rot3.Expmap(acc_omegaIMU(4:6) * deltaT);
accelGlobal = initialPoseGlobal.rotation().rotate(Point3(acc_omegaIMU(1:3))).vector;

finalPosition = Point3(initialPoseGlobal.translation.vector ...
    + initialVelocityGlobal * deltaT + 0.5 * accelGlobal * deltaT * deltaT);
finalVelocityGlobal = initialVelocityGlobal + accelGlobal * deltaT;
finalRotation = initialPoseGlobal.rotation.compose(imu2in1);
finalPose = Pose3(finalRotation, finalPosition);

end

