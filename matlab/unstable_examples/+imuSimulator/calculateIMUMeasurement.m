function [ acc_omega ] = calculateIMUMeasurement(omega1Body, omega2Body, velocity1Body, velocity2Body, deltaT, imuFrame)
%CALCULATEIMUMEASUREMENT Calculate measured body frame acceleration in
%frame 1 and measured angular rates in frame 1.

import gtsam.*;

% Calculate gyro measured rotation rate by transforming body rotation rate
% into the IMU frame.
measuredOmega = imuFrame.rotation.unrotate(Point3(omega1Body));

% Transform both velocities into IMU frame, accounting for the velocity
% induced by rigid body rotation on a lever arm (Coriolis effect).
velocity1inertial = imuFrame.rotation.unrotate( ...
    Point3(velocity1Body + cross(omega1Body, imuFrame.translation)));

imu2in1 = Rot3.Expmap(measuredOmega * deltaT);
velocity2inertial = imu2in1.rotate(imuFrame.rotation.unrotate( ...
    Point3(velocity2Body + cross(omega2Body, imuFrame.translation))));

% Acceleration in IMU frame
measuredAcc = (velocity2inertial - velocity1inertial) / deltaT;

acc_omega = [ measuredAcc; measuredOmega ];

end

