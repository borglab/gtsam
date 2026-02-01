import gtsam.*;

deltaT = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant global velocity w/ lever arm
disp('--------------------------------------------------------');
disp('Constant global velocity w/ lever arm');
omega = [0;0;0.1];
velocity = [1;0;0];
R = Rot3.Expmap(omega * deltaT);

velocity2body = R.unrotate(Point3(velocity));
acc_omegaExpected = [-0.01; 0; 0; 0; 0; 0.1];
acc_omegaActual = imuSimulator.calculateIMUMeasurement(omega, omega, velocity, velocity2body, deltaT, Pose3(Rot3, Point3([1;0;0])));
disp('IMU measurement discrepancy:');
disp(acc_omegaActual - acc_omegaExpected);

initialPose = Pose3;
finalPoseExpected = Pose3(Rot3.Expmap(omega * deltaT), Point3(velocity * deltaT));
finalVelocityExpected = velocity;
[ finalPoseActual, finalVelocityActual ] = imuSimulator.integrateTrajectory(initialPose, omega, velocity, velocity2body, deltaT);
disp('Final pose discrepancy:');
disp(finalPoseExpected.between(finalPoseActual).matrix);
disp('Final velocity discrepancy:');
disp(finalVelocityActual - finalVelocityExpected);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant body velocity w/ lever arm
disp('--------------------------------------------------------');
disp('Constant body velocity w/ lever arm');
omega = [0;0;0.1];
velocity = [1;0;0];

acc_omegaExpected = [-0.01; 0.1; 0; 0; 0; 0.1];
acc_omegaActual = imuSimulator.calculateIMUMeasurement(omega, omega, velocity, velocity, deltaT, Pose3(Rot3, Point3([1;0;0])));
disp('IMU measurement discrepancy:');
disp(acc_omegaActual - acc_omegaExpected);

initialPose = Pose3;
initialVelocity = velocity;
finalPoseExpected = Pose3.Expmap([ omega; velocity ] * deltaT);
finalVelocityExpected = finalPoseExpected.rotation.rotate(Point3(velocity));
[ finalPoseActual, finalVelocityActual ] = imuSimulator.integrateTrajectory(initialPose, omega, velocity, velocity, deltaT);
disp('Final pose discrepancy:');
disp(finalPoseExpected.between(finalPoseActual).matrix);
disp('Final velocity discrepancy:');
disp(finalVelocityActual - finalVelocityExpected);