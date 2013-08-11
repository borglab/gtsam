%close all
%clc

import gtsam.*;

%% Read metadata and compute relative sensor pose transforms
IMU_metadata = importdata('KittiEquivBiasedImu_metadata.txt');
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
                          IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

VO_metadata = importdata('KittiRelativePose_metadata.txt');
VO_metadata = cell2struct(num2cell(VO_metadata.data), VO_metadata.colheaders, 2);
VOinBody = Pose3.Expmap([VO_metadata.BodyPtx; VO_metadata.BodyPty; VO_metadata.BodyPtz;
                         VO_metadata.BodyPrx; VO_metadata.BodyPry; VO_metadata.BodyPrz; ]);

GPS_metadata = importdata('KittiGps_metadata.txt');
GPS_metadata = cell2struct(num2cell(GPS_metadata.data), GPS_metadata.colheaders, 2);
GPSinBody = Pose3.Expmap([GPS_metadata.BodyPtx; GPS_metadata.BodyPty; GPS_metadata.BodyPtz;
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
relposes = arrayfun(@(x) {gtsam.Pose3.Expmap(x{:}')}, logposes);
relposes = arrayfun(@(x) {VOinIMU.compose(x{:}).compose(VOinIMU.inverse())}, relposes);
[VO_data.RelativePose] = deal(relposes{:});
VO_data = rmfield(VO_data, { 'dtx' 'dty' 'dtz' 'drx' 'dry' 'drz' });
clear logposes relposes

GPS_data = importdata('KittiGps.txt');
GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);
  
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
sigma_init_x = 1.0;
sigma_init_v = 1.0;
sigma_init_b = 1.0;

values.insert(symbol('x',0), currentPoseGlobal);
values.insert(symbol('v',0), LieVector(currentVelocityGlobal) );
values.insert(symbol('b',0), imuBias.ConstantBias(bias_acc,bias_omega) );

disp('TODO: we have GPS so this initialization is not right')
% Prior on initial pose
factors.add(PriorFactorPose3(symbol('x',0), ...
  currentPoseGlobal, noiseModel.Isotropic.Sigma(6, sigma_init_x)));
% Prior on initial velocity
factors.add(PriorFactorLieVector(symbol('v',0), ...
  LieVector(currentVelocityGlobal), noiseModel.Isotropic.Sigma(3, sigma_init_v)));
% Prior on initial bias
factors.add(PriorFactorConstantBias(symbol('b',0), ...
  imuBias.ConstantBias(bias_acc,bias_omega), noiseModel.Isotropic.Sigma(6, sigma_init_b)));

currentSummarizedMeasurement = [];

% Measurement types:
%   1: VO
%   2: GPS
%   3: IMU
timestamps = sortrows( [ ...
  [VO_data.Time]' 1*ones(length([VO_data.Time]), 1); ...
  %[GPS_data.Time]' 2*ones(length([GPS_data.Time]), 1); ...
  %[IMU_data.Time]' 3*ones(length([IMU_data.Time]), 1); ...
  ], 1); % this are the time-stamps at which we want to initialize a new node in the graph

%% Main loop:
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory

% t_previous = 0;%LC
% poseIndex = 0;%LC 

% currentPose = Pose3; %LC

position= []; %LC

for measurementIndex = 1:size(timestamps,1)
  
  measurementIndex
  
%   currentPose = currentPose.compose(VO_data(measurementIndex).RelativePose);%LC
%   
%   position(measurementIndex,:) = currentPose.translation.vector;%LC

  
  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',poseIndex);
  currentVelKey = symbol('v',poseIndex);
  currentBiasKey = symbol('b',poseIndex);
  
  t = timestamps(measurementIndex, 1);
  type = timestamps(measurementIndex, 2);
  
%   if type == 3
%     % Integrate IMU
%     
%     if isempty(currentSummarizedMeasurement)
%       % Create initial empty summarized measurement
%       % we assume that each row of the IMU.txt file has the following structure:
%       % timestamp delta_t acc_x acc_y acc_z omega_x omega_y omega_z
%       currentBias = isam.calculateEstimate(currentBiasKey - 1);
%       currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
%         currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
%         IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
%     end
%     
%     % Accumulate preintegrated measurement
%     deltaT = IMU_data(index).dt;
%     accMeas = IMU_data(index).acc_omega(1:3);
%     omegaMeas = IMU_data(index).acc_omega(4:6);
%     currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
%     
%   else
%     % Create IMU factor
%     factors.add(ImuFactor( ...
%       currentPoseKey-1, currentVelKey-1, ...
%       currentPoseKey, currentVelKey, ...
%       currentBiasKey-1, currentSummarizedMeasurement, g, cor_v, ...
%       currentSummarizedMeasurement.PreintMeasCov));
% 
%     % Reset summarized measurement
%     currentSummarizedMeasurement = [];
%     
%     if type == 1
%       % Create VO factor
%     elseif type == 2
%       % Create GPS factor
%     end
%     
%     poseIndex = poseIndex + 1;
%   end
  
 
  % =======================================================================
  
  
  %% add factor corresponding to GPS measurements (if available at the current time)
%   % =======================================================================
%   if isempty(  find(GPS_data(:,1) == t  ) ) == 0 % it is a GPS measurement
%     if length( find(GPS_data(:,1)) ) > 1
%       error('more GPS measurements at the same time stamp: it should be an error')
%     end
%     
%     index = find(GPS_data(:,1) == t ); % the row of the IMU_data matrix that we have to integrate
%     GPSmeas = GPS_data(index,2:4);
%     
%     noiseModelGPS = ???; % noiseModelGPS.Isotropic.Sigma(6, sigma_init_x))
%     
%     % add factor
%     disp('TODO: is the GPS noise right?')
%     factors.add(PriorFactor???(currentPoseKey, GPSmeas, noiseModelGPS) );
%   end
  % =======================================================================
  
  
%   %% add factor corresponding to VO measurements (if available at the current time)
%   % =======================================================================
%   if isempty(  find([VO_data.Time] == t, 1)  )== 0 % it is a GPS measurement
%     if length( find([VO_data.Time] == t) ) > 1
%       error('more VO measurements at the same time stamp: it should be an error')
%     end
%     
%     index = find([VO_data.Time] == t, 1); % the row of the IMU_data matrix that we have to integrate
%     
%     VOpose = VO_data(index).RelativePose;
%     noiseModelVO = noiseModel.Diagonal.Sigmas([ IMU_metadata.RotationSigma * [1;1;1]; IMU_metadata.TranslationSigma * [1;1;1] ]);
%     
%     % add factor
%     disp('TODO: is the VO noise right?')
%     factors.add(BetweenFactorPose3(lastVOPoseKey, currentPoseKey, VOpose, noiseModelVO));
%     
%     lastVOPoseKey = currentPoseKey;
%   end
%   % =======================================================================
%   
%   disp('TODO: add values')
%   %     values.insert(, initialPose);
%   %   values.insert(symbol('v',lastIndex+1), initialVel);
%   
%   %% Update solver
%   % =======================================================================
%   isam.update(factors, values);
%   factors = NonlinearFactorGraph;
%   values = Values;
%   
%   isam.calculateEstimate(currentPoseKey);
%   %   M = isam.marginalCovariance(key_pose);
%   % =======================================================================
%   
%   previousPoseKey = currentPoseKey;
%   previousVelKey = currentVelKey;
%   t_previous = t;
end

figure
plot(position(:,1),position(:,2))


% figure(1)
% hold on;
% plot(positions(1,:), positions(2,:), '-b');
% plot3DTrajectory(isam.calculateEstimate, 'g-');
% axis equal;
% legend('true trajectory', 'traj integrated in body', 'traj integrated in nav')
