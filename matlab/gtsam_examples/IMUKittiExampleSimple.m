%close all
%clc

import gtsam.*;

%% Read data
IMU_metadata = importdata(gtsam.findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_data = importdata(gtsam.findExampleDataFile('KittiEquivBiasedImu.txt'));
% Make text file column headers into struct fields
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);

GPS_metadata = importdata(gtsam.findExampleDataFile('KittiGps_metadata.txt'));
GPS_data = importdata(gtsam.findExampleDataFile('KittiGps.txt'));
% Make text file column headers into struct fields
GPS_metadata = cell2struct(num2cell(GPS_metadata.data), GPS_metadata.colheaders, 2);
GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);

%% Convert GPS from lat/long to meters
[ x, y, ~ ] = deg2utm( [GPS_data.Latitude], [GPS_data.Longitude] );
for i = 1:numel(x)
    GPS_data(i).Position = gtsam.Point3(x(i), y(i), GPS_data(i).Altitude);
end

% % Calculate GPS sigma in meters
% [ xSig, ySig, ~ ] = deg2utm( [GPS_data.Latitude] + [GPS_data.PositionSigma], ...
%     [GPS_data.Longitude] + [GPS_data.PositionSigma]);
% xSig = xSig - x;
% ySig = ySig - y;

%% Start at time of first GPS measurement
firstGPSPose = 2;
  
%% Get initial conditions for the estimated trajectory
currentPoseGlobal = Pose3(Rot3, GPS_data(firstGPSPose).Position); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('QR');
isamParams.setRelinearizeSkip(1);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Create initial estimate and prior on initial pose, velocity, and biases
newValues.insert(symbol('x',firstGPSPose), currentPoseGlobal);
newValues.insert(symbol('v',firstGPSPose), currentVelocityGlobal);
newValues.insert(symbol('b',1), currentBias);

sigma_init_x = noiseModel.Diagonal.Precisions([0;0;0; 1;1;1]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigma(6, 100000.0);

newFactors.add(PriorFactorPose3(symbol('x',firstGPSPose), currentPoseGlobal, sigma_init_x));
newFactors.add(PriorFactorLieVector(symbol('v',firstGPSPose), currentVelocityGlobal, sigma_init_v));
newFactors.add(PriorFactorConstantBias(symbol('b',1), currentBias, sigma_init_b));

%% Main loop:
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory

for poseIndex = firstGPSPose:length(GPS_data)
  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',poseIndex);
  currentVelKey = symbol('v',poseIndex);
  currentBiasKey = symbol('b',1);

  if poseIndex > firstGPSPose
      % Summarize IMU data between the previous GPS measurement and now
      IMUindices = find([IMU_data.Time] > GPS_data(poseIndex-1).Time ...
          & [IMU_data.Time] <= GPS_data(poseIndex).Time);
      
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
          currentBiasKey, currentSummarizedMeasurement, [0;0;-9.8], [0;0;0]));
      
      % Create GPS factor
      newFactors.add(PriorFactorPose3(currentPoseKey, Pose3(currentPoseGlobal.rotation, GPS_data(poseIndex).Position), ...
          noiseModel.Diagonal.Precisions([ zeros(3,1); 1./(GPS_data(poseIndex).PositionSigma).^2*ones(3,1) ])));
      
      % Add initial value
      newValues.insert(currentPoseKey, Pose3(currentPoseGlobal.rotation, GPS_data(poseIndex).Position));
      newValues.insert(currentVelKey, currentVelocityGlobal);
      %newValues.insert(currentBiasKey, currentBias);

      % Update solver
      % =======================================================================
      isam.update(newFactors, newValues);
      newFactors = NonlinearFactorGraph;
      newValues = Values;

      cla;
      plot3DTrajectory(isam.calculateEstimate, 'g-');
      drawnow;
      % =======================================================================
      
      currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
      currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
      currentBias = isam.calculateEstimate(currentBiasKey);
      
  end
end

disp('TODO: display results')
% figure(1)
% hold on;
% plot(positions(1,:), positions(2,:), '-b');
% plot3DTrajectory(isam.calculateEstimate, 'g-');
% axis equal;
% legend('true trajectory', 'traj integrated in body', 'traj integrated in nav')
