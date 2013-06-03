function [ acc_omega ] = calculateIMUMeas_coriolis(omega1Body, omega2Body, velocity1Body, velocity2Body, deltaT, imuFrame)
%CALCULATEIMUMEASUREMENT Calculate measured body frame acceleration in
%frame 1 and measured angular rates in frame 1.

import gtsam.*;

% Calculate gyro measured rotation rate by transforming body rotation rate
% into the IMU frame.
measuredOmega = omega1Body;

% body2in1 = Rot3.Expmap(omega1Body * deltaT);
% % Velocity 2 in frame 1: v^1_2 = R^1_2 v^2_2
% velocity2inertial = body2in1.rotate(Point3(velocity2Body)).vector;
% % Acceleration: a^1 = (v^1_2 - v^1_1)/dt
% accelBody1 = (velocity2inertial - velocity1Body) / deltaT

% Acceleration in IMU frame
measuredAcc = Point3(cross(omega1Body, velocity1Body)).vector;
acc_omega = [ measuredAcc; measuredOmega ];

end

