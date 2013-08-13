close all
clc

import gtsam.*;
disp('Example of application of ISAM2 for visual-inertial navigation on the KITTI VISION BENCHMARK SUITE (http://www.computervisiononline.com/dataset/kitti-vision-benchmark-suite)')

%% Read metadata and compute relative sensor pose transforms
% IMU metadata
disp('-- Reading sensor metadata')
IMU_metadata = importdata('KittiEquivBiasedImu_metadata.txt');
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

% VO metadata
VO_metadata = importdata('KittiRelativePose_metadata.txt');
VO_metadata = cell2struct(num2cell(VO_metadata.data), VO_metadata.colheaders, 2);
VOinBody = Pose3.Expmap([VO_metadata.BodyPtx; VO_metadata.BodyPty; VO_metadata.BodyPtz;
  VO_metadata.BodyPrx; VO_metadata.BodyPry; VO_metadata.BodyPrz; ]);
VOinIMU = IMUinBody.inverse().compose(VOinBody);

%% Read data and change coordinate frame of GPS and VO measurements to IMU frame
disp('-- Reading sensor data from file')
% IMU data
IMU_data = importdata('KittiEquivBiasedImu.txt');
IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
imum = cellfun(@(x) x', num2cell([ [IMU_data.accelX]' [IMU_data.accelY]' [IMU_data.accelZ]' [IMU_data.omegaX]' [IMU_data.omegaY]' [IMU_data.omegaZ]' ], 2), 'UniformOutput', false);
[IMU_data.acc_omega] = deal(imum{:});
clear imum

% VO data
VO_data = importdata('KittiRelativePose.txt');
VO_data = cell2struct(num2cell(VO_data.data), VO_data.colheaders, 2);
% Merge relative pose fields and convert to Pose3
logposes = [ [VO_data.dtx]' [VO_data.dty]' [VO_data.dtz]' [VO_data.drx]' [VO_data.dry]' [VO_data.drz]' ];
logposes = num2cell(logposes, 2);
relposes = arrayfun(@(x) {gtsam.Pose3.Expmap(x{:}')}, logposes);
relposes = arrayfun(@(x) {VOinIMU.compose(x{:}).compose(VOinIMU.inverse())}, relposes);
[VO_data.RelativePose] = deal(relposes{:});
VO_data = rmfield(VO_data, { 'dtx' 'dty' 'dtz' 'drx' 'dry' 'drz' });
noiseModelVO = noiseModel.Diagonal.Sigmas([ VO_metadata.RotationSigma * [1;1;1]; VO_metadata.TranslationSigma * [1;1;1] ]);
clear logposes relposes

%% Get initial conditions for the estimated trajectory
currentPoseGlobal = Pose3;
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Sigmas([ 1.0; 1.0; 0.01; 0.01; 0.01; 0.01 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
g = [0;0;-9.8];
w_coriolis = [0;0;0];

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Main loop:
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory
timestamps = [VO_data.Time]';

timestamps = timestamps(15:end,:); % there seem to be issues with the initial IMU measurements
IMUtimes = [IMU_data.Time];

disp('-- Starting main loop: inference is performed at each time step, but we plot trajectory every 100 steps')

for measurementIndex = 1:length(timestamps)
  
  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',measurementIndex);
  currentVelKey =  symbol('v',measurementIndex);
  currentBiasKey = symbol('b',measurementIndex);
  t = timestamps(measurementIndex, 1);
  
  if measurementIndex == 1
    %% Create initial estimate and prior on initial pose, velocity, and biases
    newValues.insert(currentPoseKey, currentPoseGlobal);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
    newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
    newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
  else
    t_previous = timestamps(measurementIndex-1, 1);
    %% Summarize IMU data between the previous GPS measurement and now
    IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
    
    currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
      currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
      IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
    
    for imuIndex = IMUindices
      accMeas = [ IMU_data(imuIndex).accelX; IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ ];
      omegaMeas = [ IMU_data(imuIndex).omegaX; IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ ];
      deltaT = IMU_data(imuIndex).dt;
      currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
    end
    
    % Create IMU factor
    newFactors.add(ImuFactor( ...
      currentPoseKey-1, currentVelKey-1, ...
      currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
    
    % LC: sigma_init_b is wrong: this should be some uncertainty on bias evolution given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));
    
    %% Create VO factor
      VOpose = VO_data(measurementIndex).RelativePose;
      newFactors.add(BetweenFactorPose3(currentPoseKey - 1, currentPoseKey, VOpose, noiseModelVO));
    
    % Add initial value
    newValues.insert(currentPoseKey, currentPoseGlobal.compose(VOpose));
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    
    % Update solver
    % =======================================================================
    isam.update(newFactors, newValues);
    newFactors = NonlinearFactorGraph;
    newValues = Values;
    
    if rem(measurementIndex,100)==0 % plot every 100 time steps
      cla;
      plot3DTrajectory(isam.calculateEstimate, 'g-');
      title('Estimated trajectory using ISAM2 (IMU+VO)')
      xlabel('[m]')
      ylabel('[m]')
      zlabel('[m]')
      axis equal
      drawnow;
    end
    % ======================================================================= 
    currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
    currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
    currentBias = isam.calculateEstimate(currentBiasKey);   
  end
   
end % end main loop

disp('-- Reached end of sensor data')
