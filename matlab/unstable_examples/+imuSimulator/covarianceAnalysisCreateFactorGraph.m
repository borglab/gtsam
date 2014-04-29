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
for i=0:length(measurements)
  % Get the current pose
  currentPoseKey = symbol('x', i);
  currentPose = values.at(currentPoseKey);
  
  if i==0
    %% first time step, add priors
    % Pose prior (poses used for all factors)
    initialPose = Pose3.Expmap(measurementNoise.poseNoiseVector .* randn(6,1));
    graph.add(PriorFactorPose3(currentPoseKey, initialPose, noiseModels.noisePose));
    
    % IMU velocity and bias priors
    if options.includeIMUFactors == 1
      currentVelKey = symbol('v', 0);
      currentVel = values.at(currentVelKey).vector;
      graph.add(PriorFactorLieVector(currentVelKey, LieVector(currentVel), noiseModels.noiseVel));
      
      currentBiasKey = symbol('b', 0);
      currentBias = values.at(currentBiasKey);
      graph.add(PriorFactorConstantBias(currentBiasKey, currentBias, noiseModels.noisePriorBias));
    end
    
    % Camera priors
    if options.includeCameraFactors == 1
      pointNoiseSigma = 0.1;
      pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
      graph.add(PriorFactorPoint3(symbol('p',1), gtLandmarkPoints(1), pointPriorNoise));
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
          % TO-DO probably want to do some type of filtering on the measurement values, because
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

end % end of function

