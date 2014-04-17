function [values, measurements] = covarianceAnalysisCreateTrajectory( options, metadata )
% Create a trajectory for running covariance analysis scripts.
% 'options' contains fields for including various factor types and setting trajectory length
% 'metadata' is a storage variable for miscellaneous factor-specific values
% Authors: Luca Carlone, David Jensen
% Date: 2014/04/16

import gtsam.*;

values = Values;

    warning('fake angles! TODO: use constructor from roll-pitch-yaw when using real data')
    warning('using identity rotation')

if options.useRealData == 1
  %% Create a ground truth trajectory from Real data (if available)
  fprintf('\nUsing real data as ground truth\n');
  gtScenario = load('truth_scen2.mat', 'Time', 'Lat', 'Lon', 'Alt', 'Roll', 'Pitch', 'Heading',...
    'VEast', 'VNorth', 'VUp');
  
  Org_lat = gtScenario.Lat(1);
  Org_lon = gtScenario.Lon(1);
  initialPositionECEF = imuSimulator.LatLonHRad_to_ECEF([gtScenario.Lat(1); gtScenario.Lon(1); gtScenario.Alt(1)]);
  
  % Limit the trajectory length
  options.trajectoryLength = min([length(gtScenario.Lat) options.trajectoryLength+1]);
  fprintf('Scenario Ind: ');
  for i=1:options.trajectoryLength+1
    % Update the pose key
    currentPoseKey = symbol('x', i-1);
    
    % Generate the current pose
    scenarioInd = options.subsampleStep * (i-1) + 1;
    fprintf('%d, ', scenarioInd);
    if mod(i,20) == 0
      fprintf('\n');
    end
    gtECEF = imuSimulator.LatLonHRad_to_ECEF([gtScenario.Lat(scenarioInd); gtScenario.Lon(scenarioInd); gtScenario.Alt(scenarioInd)]);
    % truth in ENU
    dX = gtECEF(1) - initialPositionECEF(1);
    dY = gtECEF(2) - initialPositionECEF(2);
    dZ = gtECEF(3) - initialPositionECEF(3);
    [xlt, ylt, zlt] = imuSimulator.ct2ENU(dX, dY, dZ,Org_lat, Org_lon);
    
    gtPosition = [xlt, ylt, zlt]';
    gtRotation = Rot3; %Rot3.ypr(gtScenario.Heading(scenarioInd), gtScenario.Pitch(scenarioInd), gtScenario.Roll(scenarioInd));
    currentPose = Pose3(gtRotation, Point3(gtPosition));
    
    % Add values
    values.insert(currentPoseKey, currentPose);
    
    % Generate the measurement. The first pose is considered the prior, so
    % it has no measurement
    if i > 1
      prevPose = values.at(currentPoseKey - 1);
      deltaPose = prevPose.between(currentPose);
      measurements.deltaMatrix(i-1,:) = Pose3.Logmap(deltaPose);
    end
  end
  fprintf('\n');
else
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

%% Create IMU measurements and Values for the trajectory
if options.includeIMUFactors == 1
  currentVel = [0 0 0];  % initial velocity (used to generate IMU measurements)
  deltaT = 0.1;          % amount of time between IMU measurements
  
  % Iterate over the deltaMatrix to generate appropriate IMU measurements
  for i = 0:size(measurements.deltaMatrix, 1)
    % Update Keys
    currentVelKey = symbol('v', i);
    currentBiasKey = symbol('b', i);
    
    if i == 0
      % Add initial values
      currentVel = [0 0 0];
      values.insert(currentVelKey, LieVector(currentVel'));
      values.insert(currentBiasKey, metadata.imu.zeroBias);
    else
      measurements.imu.deltaT(i) = deltaT;
      
      % create accel and gyro measurements based on
      measurements.imu.gyro(i,:) = measurements.deltaMatrix(i, 1:3)./measurements.imu.deltaT(i);
      
      % acc = (deltaPosition - initialVel * dT) * (2/dt^2)
      measurements.imu.accel(i,:) = (measurements.deltaMatrix(i, 4:6) ...
        - currentVel.*measurements.imu.deltaT(i)).*(2/(measurements.imu.deltaT(i)*measurements.imu.deltaT(i)));
      
      % Update velocity
      currentVel = measurements.deltaMatrix(i,4:6)./measurements.imu.deltaT(i);
      
      % Add Values: velocity and bias
      values.insert(currentVelKey, LieVector(currentVel'));
      values.insert(currentBiasKey, metadata.imu.zeroBias);
    end
  end
end % end of IMU measurements

end % end of function

