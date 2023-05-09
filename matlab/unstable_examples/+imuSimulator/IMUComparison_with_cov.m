close all
clc

import gtsam.*;

deltaT = 0.001;
summarizedDeltaT = 0.1;
timeElapsed = 1;
times = 0:deltaT:timeElapsed;

omega = [0;0;2*pi];
velocity = [1;0;0];

g = [0;0;0];
cor_v = [0;0;0];

summaryTemplate = gtsam.ImuFactorPreintegratedMeasurements( ...
    gtsam.imuBias.ConstantBias([0;0;0], [0;0;0]), ...
    1e-3 * eye(3), 1e-3 * eye(3), 1e-3 * eye(3));

%% Set initial conditions for the true trajectory and for the estimates
% (one estimate is obtained by integrating in the body frame, the other
% by integrating in the navigation frame)
% Initial state (body)
currentPoseGlobal = Pose3;
currentVelocityGlobal = velocity;

%% Prepare data structures for actual trajectory and estimates
% Actual trajectory
positions = zeros(3, length(times)+1);
positions(:,1) = currentPoseGlobal.translation;
poses(1).p = positions(:,1);
poses(1).R = currentPoseGlobal.rotation.matrix;

%% Solver object
isamParams = ISAM2Params;
isamParams.setRelinearizeSkip(1);
isam = gtsam.ISAM2(isamParams);

sigma_init_x = 1.0;
sigma_init_v = 1.0;
sigma_init_b = 1.0;

initialValues = Values;
initialValues.insert(symbol('x',0), currentPoseGlobal);
initialValues.insert(symbol('v',0), LieVector(currentVelocityGlobal));
initialValues.insert(symbol('b',0), imuBias.ConstantBias([0;0;0],[0;0;0]));
initialFactors = NonlinearFactorGraph;
% Prior on initial pose
initialFactors.add(PriorFactorPose3(symbol('x',0), ...
    currentPoseGlobal, noiseModel.Isotropic.Sigma(6, sigma_init_x)));
% Prior on initial velocity 
initialFactors.add(PriorFactorLieVector(symbol('v',0), ...
    LieVector(currentVelocityGlobal), noiseModel.Isotropic.Sigma(3, sigma_init_v)));
% Prior on initial bias
initialFactors.add(PriorFactorConstantBias(symbol('b',0), ...
    imuBias.ConstantBias([0;0;0],[0;0;0]), noiseModel.Isotropic.Sigma(6, sigma_init_b)));

%% Main loop
i = 2;
lastSummaryTime = 0;
lastSummaryIndex = 0;
currentSummarizedMeasurement = ImuFactorPreintegratedMeasurements(summaryTemplate);
for t = times
  %% Create the ground truth trajectory, using the velocities and accelerations in the inertial frame to compute the positions
  [ currentPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
    currentPoseGlobal, omega, velocity, velocity, deltaT);
  
  %% Simulate IMU measurements, considering Coriolis effect 
  % (in this simple example we neglect gravity and there are no other forces acting on the body)
  acc_omega = imuSimulator.calculateIMUMeas_coriolis( ...
    omega, omega, velocity, velocity, deltaT);

  %% Accumulate preintegrated measurement
  currentSummarizedMeasurement.integrateMeasurement(acc_omega(1:3), acc_omega(4:6), deltaT);
  
  %% Update solver
  if t - lastSummaryTime >= summarizedDeltaT
    
      % Create IMU factor
      initialFactors.add(ImuFactor( ...
          symbol('x',lastSummaryIndex), symbol('v',lastSummaryIndex), ...
          symbol('x',lastSummaryIndex+1), symbol('v',lastSummaryIndex+1), ...
          symbol('b',0), currentSummarizedMeasurement, g, cor_v, ...
          noiseModel.Isotropic.Sigma(9, 1e-6)));
      
      % Predict movement in a straight line (bad initialization)
      if lastSummaryIndex > 0
          initialPose = isam.calculateEstimate(symbol('x',lastSummaryIndex)) ...
              .compose(Pose3(Rot3, Point3(  velocity * t - lastSummaryTime)  ));
          initialVel = isam.calculateEstimate(symbol('v',lastSummaryIndex));
      else
          initialPose = Pose3;
          initialVel = LieVector(velocity);
      end
      initialValues.insert(symbol('x',lastSummaryIndex+1), initialPose);
      initialValues.insert(symbol('v',lastSummaryIndex+1), initialVel); 
      
      key_pose = symbol('x',lastSummaryIndex+1);
      
      % Update solver
      isam.update(initialFactors, initialValues);
      initialFactors = NonlinearFactorGraph;
      initialValues = Values;
      
       isam.calculateEstimate(key_pose);
       M = isam.marginalCovariance(key_pose);
      
      lastSummaryIndex = lastSummaryIndex + 1;
      lastSummaryTime = t;
      currentSummarizedMeasurement = ImuFactorPreintegratedMeasurements(summaryTemplate);
  end
  
  %% Store data in some structure for statistics and plots
  positions(:,i) = currentPoseGlobal.translation;
  i = i + 1;
end

figure(1)
hold on;
plot(positions(1,:), positions(2,:), '-b');
plot3DTrajectory(isam.calculateEstimate, 'g-');


axis equal;
legend('true trajectory', 'traj integrated in body', 'traj integrated in nav')


