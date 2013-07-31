%close all
%clc

import gtsam.*;

%% Read metadata and compute relative sensor pose transforms
IMU_metadata = importdata('KittiEquivBiasedImu_metadata.txt');
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([
    IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
    IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);

VO_metadata = importdata('KittiRelativePose_metadata.txt');
VO_metadata = cell2struct(num2cell(VO_metadata.data), VO_metadata.colheaders, 2);
VOinBody = Pose3.Expmap([
    VO_metadata.BodyPtx; VO_metadata.BodyPty; VO_metadata.BodyPtz;
    VO_metadata.BodyPrx; VO_metadata.BodyPry; VO_metadata.BodyPrz; ]);

GPS_metadata = importdata('KittiGps_metadata.txt');
GPS_metadata = cell2struct(num2cell(GPS_metadata.data), GPS_metadata.colheaders, 2);
GPSinBody = Pose3.Expmap([
    GPS_metadata.BodyPtx; GPS_metadata.BodyPty; GPS_metadata.BodyPtz;
    GPS_metadata.BodyPrx; GPS_metadata.BodyPry; GPS_metadata.BodyPrz; ]);

VOinIMU = IMUinBody.inverse().compose(VOinBody);
GPSinIMU = IMUinBody.inverse().compose(GPSinBody);

%% Read data and change coordinate frame of GPS and VO measurements to IMU frame
IMU_data = importdata('KittiEquivBiasedImu.txt');
IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
imum = cellfun(@(x) x', num2cell([ [IMU_data.accelX]' [IMU_data.accelY]' [IMU_data.accelZ]' [IMU_data.omegaX]' [IMU_data.omegaY]' [IMU_data.omegaZ]' ], 2), 'UniformOutput', false);
[IMU_data.acc_omega] = deal(imum{:});
IMU_data = rmfield(IMU_data, { 'accelX' 'accelY' 'accelZ' 'omegaX' 'omegaY' 'omegaZ' });
clear imum

VO_data = importdata('KittiRelativePose.txt');
VO_data = cell2struct(num2cell(VO_data.data), VO_data.colheaders, 2);
% Merge relative pose fields and convert to Pose3
logposes = [ [VO_data.dtx]' [VO_data.dty]' [VO_data.dtz]' [VO_data.drx]' [VO_data.dry]' [VO_data.drz]' ];
logposes = num2cell(logposes, 2);
relposes = arrayfun(@(x) {gtsam.Pose3.Expmap(x{1}')}, logposes);
% TODO: convert to IMU frame %relposes = arrayfun(
[VO_data.RelativePose] = deal(relposes{:});
VO_data = rmfield(VO_data, { 'dtx' 'dty' 'dtz' 'drx' 'dry' 'drz' });
clear logposes relposes

GPS_data = importdata('KittiGps.txt');
GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);

%%
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
