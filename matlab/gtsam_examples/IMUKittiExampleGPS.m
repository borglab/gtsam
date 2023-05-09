import gtsam.*;
disp('Example of application of ISAM2 for GPS-aided navigation on the KITTI VISION BENCHMARK SUITE (http://www.computervisiononline.com/dataset/kitti-vision-benchmark-suite)')

%% Read metadata and compute relative sensor pose transforms
% IMU metadata
disp('-- Reading sensor metadata')
IMU_metadata = importdata(findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

%% Read data
disp('-- Reading sensor data from file')
% IMU data
IMU_data = importdata(findExampleDataFile('KittiEquivBiasedImu.txt'));
IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
imum = cellfun(@(x) x', num2cell([ [IMU_data.accelX]' [IMU_data.accelY]' [IMU_data.accelZ]' [IMU_data.omegaX]' [IMU_data.omegaY]' [IMU_data.omegaZ]' ], 2), 'UniformOutput', false);
[IMU_data.acc_omega] = deal(imum{:});
clear imum

% GPS data
GPS_data = importdata(findExampleDataFile('KittiGps_converted.txt'));
GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);
for i = 1:numel(GPS_data)
    GPS_data(i).Position = gtsam.Point3(GPS_data(i).X, GPS_data(i).Y, GPS_data(i).Z);
end
noiseModelGPS = noiseModel.Diagonal.Precisions([ [0;0;0]; 1.0/0.07 * [1;1;1] ]);
firstGPSPose = 2;
GPSskip = 10; % Skip this many GPS measurements each time

%% Get initial conditions for the estimated trajectory
currentPoseGlobal = Pose3(Rot3, GPS_data(firstGPSPose).Position); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = [0;0;0]; % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
g = [0;0;-9.8];
w_coriolis = [0;0;0];

IMU_params = PreintegrationParams(g);

IMU_params.setAccelerometerCovariance(IMU_metadata.AccelerometerSigma.^2 * eye(3));
IMU_params.setGyroscopeCovariance(IMU_metadata.GyroscopeSigma.^2 * eye(3));
IMU_params.setIntegrationCovariance(IMU_metadata.IntegrationSigma.^2 * eye(3));
IMU_params.setOmegaCoriolis(w_coriolis);

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
IMUtimes = [IMU_data.Time];

disp('-- Starting main loop: inference is performed at each time step, but we plot trajectory every 10 steps')

for measurementIndex = firstGPSPose:length(GPS_data)
  
  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',measurementIndex);
  currentVelKey =  symbol('v',measurementIndex);
  currentBiasKey = symbol('b',measurementIndex);
  t = GPS_data(measurementIndex, 1).Time;
     
  if measurementIndex == firstGPSPose
    %% Create initial estimate and prior on initial pose, velocity, and biases
    newValues.insert(currentPoseKey, currentPoseGlobal);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
    newFactors.add(PriorFactorVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
    newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
  else
    t_previous = GPS_data(measurementIndex-1, 1).Time;
    %% Summarize IMU data between the previous GPS measurement and now
    IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
        
    currentSummarizedMeasurement = PreintegratedImuMeasurements(IMU_params,currentBias);
    
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
      currentBiasKey-1, currentSummarizedMeasurement));

    % Bias evolution as given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));

    % Create GPS factor
    GPSPose = Pose3(currentPoseGlobal.rotation, GPS_data(measurementIndex).Position);
    if mod(measurementIndex, GPSskip) == 0
      newFactors.add(PriorFactorPose3(currentPoseKey, GPSPose, noiseModelGPS));
    end

    % Add initial value
    newValues.insert(currentPoseKey, GPSPose);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    
    % Update solver
    % =======================================================================
    % We accumulate 2*GPSskip GPS measurements before updating the solver at
    % first so that the heading becomes observable.
    if measurementIndex > firstGPSPose + 2*GPSskip
      isam.update(newFactors, newValues);
      newFactors = NonlinearFactorGraph;
      newValues = Values;

      result = isam.calculateEstimate();
      
      if rem(measurementIndex,10)==0 % plot every 10 time steps
        cla;
        
        plot3DTrajectory(result, 'g-');
        title('Estimated trajectory using ISAM2 (IMU+GPS)')
        xlabel('[m]')
        ylabel('[m]')
        zlabel('[m]')
        axis equal
        drawnow;
      end
      % =======================================================================

      currentPoseGlobal = result.atPose3(currentPoseKey);
      currentVelocityGlobal = result.atVector(currentVelKey);
      currentBias = result.atConstantBias(currentBiasKey);
    end
  end
     
end % end main loop

disp('-- Reached end of sensor data')
