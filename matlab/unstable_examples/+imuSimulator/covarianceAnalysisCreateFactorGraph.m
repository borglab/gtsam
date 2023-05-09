function [ graph projectionFactorSeenBy] = covarianceAnalysisCreateFactorGraph( measurements, values, noiseModels, measurementNoise, options, metadata)
% Create a factor graph based on provided measurements, values, and noises.
% Used for covariance analysis scripts.
% 'options' contains fields for including various factor types.
% 'metadata' is a storage variable for miscellaneous factor-specific values
% Authors: Luca Carlone, David Jensen
% Date: 2014/04/16

import gtsam.*;

graph = NonlinearFactorGraph;

% Iterate through the trajectory
for i=0:length(measurements)
  % Get the current pose
  currentPoseKey = symbol('x', i);
  currentPose = values.atPose3(currentPoseKey);
  
  if i==0
    %% first time step, add priors
    % Pose prior (poses used for all factors)
    noisyInitialPoseVector = Pose3.Logmap(currentPose) + measurementNoise.poseNoiseVector .* randn(6,1); 
    initialPose = Pose3.Expmap(noisyInitialPoseVector);
    graph.add(PriorFactorPose3(currentPoseKey, initialPose, noiseModels.noisePose));
    
    % IMU velocity and bias priors
    if options.includeIMUFactors == 1
      currentVelKey = symbol('v', 0);
      currentVel = values.atPoint3(currentVelKey);
      graph.add(PriorFactorLieVector(currentVelKey, LieVector(currentVel), noiseModels.noiseVel));
      
      currentBiasKey = symbol('b', 0);
      currentBias = values.atPoint3(currentBiasKey);
      graph.add(PriorFactorConstantBias(currentBiasKey, currentBias, noiseModels.noisePriorBias));
    end
    
    %% Create a SmartProjectionFactor for each landmark
    projectionFactorSeenBy = [];
    if options.includeCameraFactors == 1
      for j=1:options.numberOfLandmarks
        SmartProjectionFactors(j) = SmartProjectionPose3Factor(0.01);
        % Use constructor with default values, but express the pose of the
        % camera as a 90 degree rotation about the X axis
%         SmartProjectionFactors(j) = SmartProjectionPose3Factor( ...
%             1, ...      % rankTol
%             -1, ...     % linThreshold
%             false, ...  % manageDegeneracy
%             false, ...  % enableEPI
%             metadata.camera.bodyPoseCamera);    % Pose of camera in body frame
      end
      projectionFactorSeenBy = zeros(options.numberOfLandmarks,1);
    end
    
  else
    
    %% Add Between factors
    if options.includeBetweenFactors == 1
      prevPoseKey = symbol('x', i-1);
      % Create the noisy pose estimate
      deltaPose = Pose3.Expmap(measurements(i).deltaVector ...
        + (measurementNoise.poseNoiseVector .* randn(6,1)));  % added noise
      % Add the between factor to the graph
      graph.add(BetweenFactorPose3(prevPoseKey, currentPoseKey, deltaPose, noiseModels.noisePose));
    end % end of Between pose factor creation
    
    %% Add IMU factors
    if options.includeIMUFactors == 1
      % Update keys
      currentVelKey = symbol('v', i);  % not used if includeIMUFactors is false
      currentBiasKey = symbol('b', i); % not used if includeIMUFactors is false
      
      if options.imuFactorType == 1
        use2ndOrderIntegration = true;
        % Initialize preintegration
        imuMeasurement = gtsam.ImuFactorPreintegratedMeasurements(...
          metadata.imu.zeroBias, ...
          metadata.imu.AccelerometerSigma.^2 * eye(3), ...
          metadata.imu.GyroscopeSigma.^2 * eye(3), ...
          metadata.imu.IntegrationSigma.^2 * eye(3), ...
          use2ndOrderIntegration);
        % Generate IMU measurements with noise
        for j=1:length(measurements(i).imu) % all measurements to preintegrate
          accelNoise = (measurementNoise.imu.accelNoiseVector .* randn(3,1));
          imuAccel = measurements(i).imu(j).accel ...
            + accelNoise ...  % added noise
            + metadata.imu.accelConstantBiasVector;     % constant bias
        
          gyroNoise = (measurementNoise.imu.gyroNoiseVector .* randn(3,1));
          imuGyro = measurements(i).imu(j).gyro ...
            + gyroNoise ...   % added noise
            + metadata.imu.gyroConstantBiasVector;      % constant bias
          
          % Used for debugging
          %fprintf('  A: (meas)[%f %f %f] + (noise)[%f %f %f] + (bias)[%f %f %f] = [%f %f %f]\n', ...
          %    measurements(i).imu(j).accel(1), measurements(i).imu(j).accel(2), measurements(i).imu(j).accel(3), ...
          %    accelNoise(1), accelNoise(2), accelNoise(3), ...
          %    metadata.imu.accelConstantBiasVector(1), metadata.imu.accelConstantBiasVector(2), metadata.imu.accelConstantBiasVector(3), ...
          %    imuAccel(1), imuAccel(2), imuAccel(3));
          %fprintf('  G: (meas)[%f %f %f] + (noise)[%f %f %f] + (bias)[%f %f %f] = [%f %f %f]\n', ...
          %    measurements(i).imu(j).gyro(1), measurements(i).imu(j).gyro(2), measurements(i).imu(j).gyro(3), ...
          %    gyroNoise(1), gyroNoise(2), gyroNoise(3), ...
          %    metadata.imu.gyroConstantBiasVector(1), metadata.imu.gyroConstantBiasVector(2), metadata.imu.gyroConstantBiasVector(3), ...
          %    imuGyro(1), imuGyro(2), imuGyro(3));
          
          % Preintegrate
          imuMeasurement.integrateMeasurement(imuAccel, imuGyro, measurements(i).imu(j).deltaT);
        end
        %imuMeasurement.print('imuMeasurement');
        
        % Add Imu factor
        graph.add(ImuFactor(currentPoseKey-1, currentVelKey-1, currentPoseKey, currentVelKey, ...
          currentBiasKey-1, imuMeasurement, metadata.imu.g, metadata.imu.omegaCoriolis));
        % Add between factor on biases
        graph.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, metadata.imu.zeroBias, ...
          noiseModels.noiseBiasBetween));
      end
          
      if options.imuFactorType == 2
        use2ndOrderIntegration = true;
        % Initialize preintegration
        imuMeasurement = gtsam.CombinedImuFactorPreintegratedMeasurements(...
          metadata.imu.zeroBias, ...
          metadata.imu.AccelerometerSigma.^2 * eye(3), ...
          metadata.imu.GyroscopeSigma.^2 * eye(3), ...
          metadata.imu.IntegrationSigma.^2 * eye(3), ...
          metadata.imu.BiasAccelerometerSigma.^2 * eye(3), ...  % how bias changes over time
          metadata.imu.BiasGyroscopeSigma.^2 * eye(3), ...      % how bias changes over time
          diag(metadata.imu.BiasAccOmegaInit.^2), ...           % prior on bias
          use2ndOrderIntegration);
        % Generate IMU measurements with noise
        for j=1:length(measurements(i).imu) % all measurements to preintegrate
          imuAccel = measurements(i).imu(j).accel ...
            + (measurementNoise.imu.accelNoiseVector .* randn(3,1))...  % added noise
            + metadata.imu.accelConstantBiasVector;     % constant bias
          imuGyro = measurements(i).imu(j).gyro ...
            + (measurementNoise.imu.gyroNoiseVector .* randn(3,1))...   % added noise
            + metadata.imu.gyroConstantBiasVector;      % constant bias
          
          % Preintegrate
          imuMeasurement.integrateMeasurement(imuAccel, imuGyro, measurements(i).imu(j).deltaT);
        end
        
        % Add Imu factor
        graph.add(CombinedImuFactor( ...
          currentPoseKey-1, currentVelKey-1, ...
          currentPoseKey, currentVelKey, ...
          currentBiasKey-1, currentBiasKey, ...
          imuMeasurement, ...
          metadata.imu.g, metadata.imu.omegaCoriolis));
      end
      
    end % end of IMU factor creation
    
    %% Build Camera Factors
    if options.includeCameraFactors == 1        
      for j = 1:length(measurements(i).landmarks)
        cameraMeasurmentNoise = measurementNoise.cameraNoiseVector .* randn(2,1);
        cameraPixelMeasurement = measurements(i).landmarks(j);
        % Only add the measurement if it is in the image frame (based on calibration)
        if(cameraPixelMeasurement(1) > 0 && cameraPixelMeasurement(2) > 0 ...
             && cameraPixelMeasurement(1) < 2*metadata.camera.calibration.px ...
             && cameraPixelMeasurement(2) < 2*metadata.camera.calibration.py)
          cameraPixelMeasurement = cameraPixelMeasurement + cameraMeasurmentNoise;
          SmartProjectionFactors(j).add( ...
             Point2(cameraPixelMeasurement), ...
             currentPoseKey, noiseModels.noiseCamera, ...
             metadata.camera.calibration);
           projectionFactorSeenBy(j) = projectionFactorSeenBy(j)+1;
        end
      end
    end % end of Camera factor creation
    
    %% Add GPS factors
    if options.includeGPSFactors == 1 && i >= options.gpsStartPose
      gpsPosition = measurements(i).gpsPositionVector ...
          + measurementNoise.gpsNoiseVector .* randn(3,1);
      graph.add(PriorFactorPose3(currentPoseKey, ...
          Pose3(currentPose.rotation, Point3(gpsPosition)), ...
          noiseModels.noiseGPS)); 
    end % end of GPS factor creation
    
  end % end of else (i=0)
  
end % end of for over trajectory

%% Add Camera Factors to the graph
% Only factors for landmarks that have been viewed at least once are added
% to the graph
%[find(projectionFactorSeenBy ~= 0) projectionFactorSeenBy(find(projectionFactorSeenBy ~= 0))]
if options.includeCameraFactors == 1
  for j = 1:options.numberOfLandmarks
    if projectionFactorSeenBy(j) > 0
      graph.add(SmartProjectionFactors(j));
    end
  end
end

end % end of function

