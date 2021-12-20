function [ acc_omega ] = calculateIMUMeas_coriolis(omega1Body, omega2Body, velocity1Body, velocity2Body, deltaT)

import gtsam.*;

% gyro measured rotation rate
measuredOmega = omega1Body;

% Acceleration measurement (in this simple toy example no other forces 
% act on the body and the only acceleration is the centripetal Coriolis acceleration)
measuredAcc = Point3(cross(omega1Body, velocity1Body));
acc_omega = [ measuredAcc; measuredOmega ];

end

