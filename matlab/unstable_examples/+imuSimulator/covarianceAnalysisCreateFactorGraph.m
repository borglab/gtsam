function [ graph ] = covarianceAnalysisCreateFactorGraph( measurements, values, noiseModels, measurementNoise, options, metadata) 
% Create a factor graph based on provided measurements, values, and noises.
% Used for covariance analysis scripts.
% 'options' contains fields for including various factor types.
% 'metadata' is a storage variable for miscellaneous factor-specific values
% Authors: Luca Carlone, David Jensen
% Date: 2014/04/16

import gtsam.*;

graph = NonlinearFactorGraph;

% Iterate through the trajectory 
for i=0:size(measurements.deltaMatrix, 1);
  % Get the current pose
  currentPoseKey = symbol('x', i);
  currentPose = values.at(currentPoseKey); 
  
  if i==0
    %% first time step, add priors
    warning('fake angles! TODO: use constructor from roll-pitch-yaw when using real data')
    warning('using identity rotation')
    
    % Pose prior (poses used for all factors)
    initialPose = Pose3.Expmap(measurementNoise.poseNoiseVector .* randn(6,1));
    graph.add(PriorFactorPose3(currentPoseKey, initialPose, noiseModels.noisePose));
    
    % IMU velocity and bias priors
    if options.includeIMUFactors == 1
      currentVelKey = symbol('v', 0);
      currentBiasKey = symbol('b', 0);
      currentVel = [0; 0; 0];
      graph.add(PriorFactorLieVector(currentVelKey, LieVector(currentVel), noiseModels.noiseVel));
      graph.add(PriorFactorConstantBias(currentBiasKey, metadata.imu.zeroBias, noiseModels.noisePriorBias));
    end
    
    % Camera priors
    if options.includeCameraFactors == 1
      pointNoiseSigma = 0.1;
      pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
      graph.add(PriorFactorPoint3(symbol('p',1), gtLandmarkPoints(1), pointPriorNoise));
    end
    
  else
    prevPoseKey = symbol('x', i-1);

    %% Add Between factors
    if options.includeBetweenFactors == 1
      % Create the noisy pose estimate
      deltaPose = Pose3.Expmap(measurements.deltaMatrix(i,:)' ...
        + (measurementNoise.poseNoiseVector .* randn(6,1)));  % added noise
      % Add the between factor to the graph
      graph.add(BetweenFactorPose3(prevPoseKey, currentPoseKey, deltaPose, noiseModels.noisePose));
    end % end of Between pose factor creation
    
    %% Add IMU factors
    if options.includeIMUFactors == 1
      % Update keys
      currentVelKey = symbol('v', i);  % not used if includeIMUFactors is false
      currentBiasKey = symbol('b', i); % not used if includeIMUFactors is false
      % Generate IMU measurements with noise
      imuAccel = measurements.imu.accel(i,:)' ...
          + (measurementNoise.imu.accelNoiseVector .* randn(3,1));  % added noise
      imuGyro = measurements.imu.gyro(i,:)' ...
          + (measurementNoise.imu.gyroNoiseVector .* randn(3,1));   % added noise

      if options.imuFactorType == 2
        % Initialize preintegration
        imuMeasurement = gtsam.CombinedImuFactorPreintegratedMeasurements(...
          metadata.imu.zeroBias, ...
          metadata.imu.AccelerometerSigma.^2 * eye(3), ...
          metadata.imu.GyroscopeSigma.^2 * eye(3), ...
          metadata.imu.IntegrationSigma.^2 * eye(3), ...
          metadata.imu.BiasAccelerometerSigma.^2 * eye(3), ...
          metadata.imu.BiasGyroscopeSigma.^2 * eye(3), ...
          metadata.imu.BiasAccOmegaInit.^2 * eye(6));
        % Preintegrate
        imuMeasurement.integrateMeasurement(imuAccel, imuGyro, measurements.imu.deltaT(i));
        % Add Imu factor
        graph.add(CombinedImuFactor( ...
          currentPoseKey-1, currentVelKey-1, ...
          currentPoseKey, currentVelKey, ...
          currentBiasKey-1, currentBiasKey, ...
          imuMeasurement, ...
          metadata.imu.g, metadata.imu.omegaCoriolis, ...
          noiseModel.Isotropic.Sigma(15, metadata.imu.epsBias)));
      else % Assumed to be type 1 if type 2 is not selected
        % Initialize preintegration
        imuMeasurement = gtsam.ImuFactorPreintegratedMeasurements(...
          metadata.imu.zeroBias, ...
          metadata.imu.AccelerometerSigma.^2 * eye(3), ...
          metadata.imu.GyroscopeSigma.^2 * eye(3), ...
          metadata.imu.IntegrationSigma.^2 * eye(3));
        % Preintegrate
        imuMeasurement.integrateMeasurement(imuAccel, imuGyro, measurements.imu.deltaT(i));
        % Add Imu factor
        graph.add(ImuFactor(currentPoseKey-1, currentVelKey-1, currentPoseKey, currentVelKey, ...
          currentBiasKey-1, imuMeasurement, metadata.imu.g, metadata.imu.omegaCoriolis));
        % Add between factor on biases
        graph.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, metadata.imu.zeroBias, ...
          noiseModel.Isotropic.Sigma(6, metadata.imu.epsBias)));
        % Additional prior on zerobias
        graph.add(PriorFactorConstantBias(currentBiasKey, metadata.imu.zeroBias, ...
          noiseModel.Isotropic.Sigma(6, metadata.imu.epsBias))); 
      end
     
    end % end of IMU factor creation
    
    %% Add Camera factors - UNDER CONSTRUCTION !!!! -
    if options.includeCameraFactors == 1
      % Create camera with the current pose and calibration K (specified above)
      gtCamera = SimpleCamera(currentPose, K);
      % Project landmarks into the camera
      numSkipped = 0;
      for j = 1:length(gtLandmarkPoints)
        landmarkKey = symbol('p', j);
        try
          Z = gtCamera.project(gtLandmarkPoints(j));
          %% TO-DO probably want to do some type of filtering on the measurement values, because
          % they might not all be valid
          graph.add(GenericProjectionFactorCal3_S2(Z, cameraMeasurementNoise, currentPoseKey, landmarkKey, K));
        catch
          % Most likely the point is not within the camera's view, which
          % is fine
          numSkipped = numSkipped + 1;
        end
      end
      %fprintf('(Pose %d) %d landmarks behind the camera\n', i, numSkipped);
    end % end of Camera factor creation
    
  end % end of else
  
end % end of for over trajectory

end

