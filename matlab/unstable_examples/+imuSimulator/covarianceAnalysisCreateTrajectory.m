function [values, measurements] = covarianceAnalysisCreateTrajectory( options, metadata )
% Create a trajectory for running covariance analysis scripts.
% 'options' contains fields for including various factor types and setting trajectory length
% 'metadata' is a storage variable for miscellaneous factor-specific values
% Authors: Luca Carlone, David Jensen
% Date: 2014/04/16

import gtsam.*;

values = Values;

warning('Rotating Pose inside getPoseFromGtScenario! TODO: Use a body_P_sensor transform in the factors')


if options.useRealData == 1
  %% Create a ground truth trajectory from Real data (if available)
  fprintf('\nUsing real data as ground truth\n');
  gtScenario = load('truth_scen2.mat', 'Time', 'Lat', 'Lon', 'Alt', 'Roll', 'Pitch', 'Heading',...
    'VEast', 'VNorth', 'VUp');
  
  % Limit the trajectory length
  options.trajectoryLength = min([length(gtScenario.Lat)/options.subsampleStep options.trajectoryLength]);
  fprintf('Scenario Ind: ');
  
  for i=0:options.trajectoryLength
    scenarioInd = options.subsampleStep * i + 1;
    fprintf('%d, ', scenarioInd);
    if (mod(i,12) == 0) fprintf('\n'); end
    
    %% Generate the current pose
    currentPoseKey = symbol('x', i);
    currentPose = imuSimulator.getPoseFromGtScenario(gtScenario,scenarioInd);
    
    %% FOR TESTING - this is currently moved to getPoseFromGtScenario
    %currentPose = currentPose.compose(metadata.camera.bodyPoseCamera);
    %currentPose = currentPose.compose(Pose3.Expmap([-pi/2;0;0;0;0;0]));
    
    % add to values
    values.insert(currentPoseKey, currentPose);
    
    %% gt Between measurements
    if options.includeBetweenFactors == 1 && i > 0
      prevPose = values.atPose3(currentPoseKey - 1);
      deltaPose = prevPose.between(currentPose);
      measurements(i).deltaVector = Pose3.Logmap(deltaPose);
    end
    
    %% gt IMU measurements
    if options.includeIMUFactors == 1
      currentVelKey = symbol('v', i);
      currentBiasKey = symbol('b', i);
      deltaT = 1;   % amount of time between IMU measurements
      if i == 0
        currentVel = [0 0 0]';
      else
        % integrate & store intermediate measurements       
        for j=1:options.subsampleStep % we integrate all the intermediate measurements
          previousScenarioInd = options.subsampleStep * (i-1) + 1;
          scenarioIndIMU1 = previousScenarioInd+j-1;
          scenarioIndIMU2 = previousScenarioInd+j;
          IMUPose1 = imuSimulator.getPoseFromGtScenario(gtScenario,scenarioIndIMU1);
          IMUPose2 = imuSimulator.getPoseFromGtScenario(gtScenario,scenarioIndIMU2);
          IMURot1 = IMUPose1.rotation.matrix;
                    
          IMUdeltaPose = IMUPose1.between(IMUPose2);
          IMUdeltaPoseVector     = Pose3.Logmap(IMUdeltaPose);
          IMUdeltaRotVector      = IMUdeltaPoseVector(1:3);
          IMUdeltaPositionVector = IMUPose2.translation - IMUPose1.translation; % translation in absolute frame
          
          measurements(i).imu(j).deltaT = deltaT;
          
          % gyro rate: Logmap(R_i' * R_i+1) / deltaT
          measurements(i).imu(j).gyro = IMUdeltaRotVector./deltaT;
          
          % deltaPij += deltaVij * deltaT + 0.5 * deltaRij.matrix() * biasHat.correctAccelerometer(measuredAcc) * deltaT*deltaT;
          % acc = (deltaPosition - initialVel * dT) * (2/dt^2)
          measurements(i).imu(j).accel = IMURot1' * (IMUdeltaPositionVector - currentVel.*deltaT).*(2/(deltaT*deltaT));
          
          % Update velocity
          currentVel = currentVel + IMURot1 * measurements(i).imu(j).accel * deltaT;
        end
      end
      
      % Add Values: velocity and bias
      values.insert(currentVelKey, currentVel);
      values.insert(currentBiasKey, metadata.imu.zeroBias);
    end
    
    %% gt GPS measurements
    if options.includeGPSFactors == 1 && i > 0
      gpsPositionVector = imuSimulator.getPoseFromGtScenario(gtScenario,scenarioInd).translation;
      measurements(i).gpsPositionVector = gpsPositionVector;
    end
    
    %% gt Camera measurements
    if options.includeCameraFactors == 1 && i > 0     
      % Create the camera based on the current pose and the pose of the
      % camera in the body
      cameraPose = currentPose.compose(metadata.camera.bodyPoseCamera);
      camera = PinholeCameraCal3_S2(cameraPose, metadata.camera.calibration);
      %camera = PinholeCameraCal3_S2(currentPose, metadata.camera.calibration);
      
      % Record measurements if the landmark is within visual range
      for j=1:length(metadata.camera.gtLandmarkPoints)
        distanceToLandmark = camera.pose.range(metadata.camera.gtLandmarkPoints(j));
        if distanceToLandmark < metadata.camera.visualRange
          try
            z = camera.project(metadata.camera.gtLandmarkPoints(j));
            measurements(i).landmarks(j) = z;
          catch
            % point is probably out of the camera's view
          end
        end
      end
    end
    
  end
  fprintf('\n');
else
  error('Please use RealData')
  %% Create a random trajectory as ground truth
  currentPose = Pose3; % initial pose  % initial pose
  
  unsmooth_DP = 0.5; % controls smoothness on translation norm
  unsmooth_DR = 0.1; % controls smoothness on rotation norm
  
  fprintf('\nCreating a random ground truth trajectory\n');
  currentPoseKey = symbol('x', 0);
  values.insert(currentPoseKey, currentPose);
  
  for i=1:options.trajectoryLength
    % Update the pose key
    currentPoseKey = symbol('x', i);
    
    % Generate the new measurements
    gtDeltaPosition = unsmooth_DP*randn(3,1) + [20;0;0]; % create random vector with mean = [20 0 0]
    gtDeltaRotation = unsmooth_DR*randn(3,1) + [0;0;0]; % create random rotation with mean [0 0 0]
    measurements.deltaMatrix(i,:) = [gtDeltaRotation; gtDeltaPosition];
    
    % Create the next pose
    deltaPose = Pose3.Expmap(measurements.deltaMatrix(i,:)');
    currentPose = currentPose.compose(deltaPose);
    
    % Add the current pose as a value
    values.insert(currentPoseKey, currentPose);
  end  % end of random trajectory creation
end % end of else

end % end of function

