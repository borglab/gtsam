close all
clc

import gtsam.*;

IMU_data = dmlread('IMU.txt');
VO_data = dmlread('VO.txt');
GPS_data = dmlread('GPS.txt');

SummaryTemplate = gtsam.ImuFactorPreintegratedMeasurements( ...
    gtsam.imuBias.ConstantBias([0;0;0], [0;0;0]), ...
    1e-3 * eye(3), 1e-3 * eye(3), 1e-3 * eye(3));
  
%% Set initial conditions for the estimated trajectory
disp('TODO: we have GPS so this initialization is not right')
currentPoseGlobal = Pose3; % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = [0;0;0]; % the vehicle is stationary at the beginning
bias_acc = [0;0;0]; % we initialize accelerometer biases to zero
bias_omega = [0;0;0]; % we initialize gyro biases to zero

%% Solver object
isamParams = ISAM2Params;
isamParams.setRelinearizeSkip(1);
isam = gtsam.ISAM2(isamParams);

%% create nonlinear factor graph
factors = NonlinearFactorGraph;
values = Values;

%% Create prior on initial pose, velocity, and biases
sigma_init_x = 1.0
sigma_init_v = 1.0
sigma_init_b = 1.0

values.insert(symbol('x',0), currentPoseGlobal);
values.insert(symbol('v',0), LieVector(currentVelocityGlobal) );
values.insert(symbol('b',0), imuBias.ConstantBias(bias_acc,bias_omega) );

% Prior on initial pose
factors.add(PriorFactorPose3(symbol('x',0), ...
  currentPoseGlobal, noiseModel.Isotropic.Sigma(6, sigma_init_x)));
% Prior on initial velocity
factors.add(PriorFactorLieVector(symbol('v',0), ...
  LieVector(currentVelocityGlobal), noiseModel.Isotropic.Sigma(3, sigma_init_v)));
% Prior on initial bias
factors.add(PriorFactorConstantBias(symbol('b',0), ...
  imuBias.ConstantBias(bias_acc,bias_omega), noiseModel.Isotropic.Sigma(6, sigma_init_b)));

%% Main loop:
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory

i = 2;
lastTime = 0;
lastIndex = 0;
currentSummarizedMeasurement = ImuFactorPreintegratedMeasurements(summaryTemplate);

times = sort([VO_data(:,1); GPS_data(:,1)]); % this are the time-stamps at which we want to initialize a new node in the graph
IMU_times = IMU_data(:,1);

disp('TODO: still needed to take care of the initial time')

for t = times
  % At each non=IMU measurement we initialize a new node in the graph
  currentIndex = find( times == t );
  currentPoseKey = symbol('x',currentIndex);
  currentVelKey = symbol('v',currentIndex);
  currentBiasKey = symbol('b',currentIndex);
  
  %% add preintegrated IMU factor between previous state and current state
  % =======================================================================
  IMUbetweenTimesIndices = find( IMU_times>+t_previous & IMU_times<= t);
  % all imu measurements occurred between t_previous and t
  
  % we assume that each row of the IMU.txt file has the following structure:
  % timestamp delta_t acc_x acc_y acc_z omega_x omega_y omega_z
  disp('TODO: We want don t want to preintegrate with zero bias, but to use the most recent estimate')
  currentSummarizedMeasurement = ImuFactorPreintegratedMeasurements(summaryTemplate);
  for i=1:length(IMUbetweenTimesIndices)
    index = IMUbetweenTimesIndices(i); % the row of the IMU_data matrix that we have to integrate
    deltaT = IMU_data(index,2);
    accMeas = IMU_data(index,3:5);
    omegaMeas = IMU_data(index,6:8);
    % Accumulate preintegrated measurement
    currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
  end
  
  disp('TODO: is the imu noise right?')
  % Create IMU factor
  factors.add(ImuFactor( ...
    previousPoseKey, previousVelKey, ...
    currentPoseKey, currentVelKey, ...
    currentBiasKey, currentSummarizedMeasurement, g, cor_v, ...
    noiseModel.Isotropic.Sigma(9, 1e-6)));
  % =======================================================================
  
  
  %% add factor corresponding to GPS measurements (if available at the current time)
  % =======================================================================
  if isempty(  find(GPS_data(:,1) == t  ) ) == 0 % it is a GPS measurement
    if length( find(GPS_data(:,1)) ) > 1
      error('more GPS measurements at the same time stamp: it should be an error')
    end
    
    index = find(GPS_data(:,1) == t ); % the row of the IMU_data matrix that we have to integrate
    GPSmeas = GPS_data(index,2:4);
    
    noiseModelGPS = ???; % noiseModelGPS.Isotropic.Sigma(6, sigma_init_x))
    
    % add factor
    disp('TODO: is the GPS noise right?')
    factors.add(PriorFactor???(currentPoseKey, GPSmeas, noiseModelGPS) );
  end
  % =======================================================================
  
  
  %% add factor corresponding to VO measurements (if available at the current time)
  % =======================================================================
  if isempty(  find(VO_data(:,1) == t  )  )== 0 % it is a GPS measurement
    if length( find(VO_data(:,1)) ) > 1
      error('more VO measurements at the same time stamp: it should be an error')
    end
    
    index = find( VO_data(:,1) == t ); % the row of the IMU_data matrix that we have to integrate
    VOmeas_pos = VO_data(index,2:4)';
    VOmeas_ang = VO_data(index,5:7)';
    
    VOpose = Pose3( Rot3(VOmeas_ang) , Point3(VOmeas_pos) );
    noiseModelVO = ???; % noiseModelGPS.Isotropic.Sigma(6, sigma_init_x))
    
    % add factor
    disp('TODO: is the VO noise right?')
    factors.add(BetweenFactorPose3(lastVOPoseKey, currentPoseKey, VOpose, noiseModelVO));
    
    lastVOPoseKey = currentPoseKey;
  end
  % =======================================================================
  
  disp('TODO: add values')
  %     values.insert(, initialPose);
  %   values.insert(symbol('v',lastIndex+1), initialVel);
  
  %% Update solver
  % =======================================================================
  isam.update(factors, values);
  factors = NonlinearFactorGraph;
  values = Values;
  
  isam.calculateEstimate(currentPoseKey);
  %   M = isam.marginalCovariance(key_pose);
  % =======================================================================
  
  previousPoseKey = currentPoseKey;
  previousVelKey = currentVelKey;
  t_previous = t;
end

disp('TODO: display results')
% figure(1)
% hold on;
% plot(positions(1,:), positions(2,:), '-b');
% plot3DTrajectory(isam.calculateEstimate, 'g-');
% axis equal;
% legend('true trajectory', 'traj integrated in body', 'traj integrated in nav')
