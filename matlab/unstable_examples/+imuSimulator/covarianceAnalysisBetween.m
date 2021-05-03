import gtsam.*;

% Test GTSAM covariances on a factor graph with:
% Between Factors
% IMU factors (type 1 and type 2)
% GPS prior factors on poses
% SmartProjectionPoseFactors
% Authors: Luca Carlone, David Jensen
% Date: 2014/4/6


% Check for an extneral configuration, used when running multiple tests
if ~exist('externallyConfigured', 'var')
  clc
  clear all
  close all
  
  saveResults = 0;
  
  %% Configuration
  % General options
  options.useRealData = 1;           % controls whether or not to use the real data (if available) as the ground truth traj
  options.includeBetweenFactors = 0; % if true, BetweenFactors will be added between consecutive poses
  
  options.includeIMUFactors = 1;     % if true, IMU factors will be added between consecutive states (biases, poses, velocities)
  options.imuFactorType = 1;         % Set to 1 or 2 to use IMU type 1 or type 2 factors (will default to type 1)
  options.imuNonzeroBias = 0;        % if true, a nonzero bias is applied to IMU measurements
  
  options.includeCameraFactors = 1;  % if true, SmartProjectionPose3Factors will be used with randomly generated landmarks
  options.numberOfLandmarks = 1000;  % Total number of visual landmarks (randomly generated in a box around the trajectory)
  
  options.includeGPSFactors = 0;     % if true, GPS factors will be added as priors to poses
  options.gpsStartPose = 100;        % Pose number to start including GPS factors at
  
  options.trajectoryLength = 100;%209;    % length of the ground truth trajectory
  options.subsampleStep = 20;        % number of poses to skip when using real data (to reduce computation on long trajectories)
  
  numMonteCarloRuns = 2;             % number of Monte Carlo runs to perform
  
  % Noise values to be adjusted
  sigma_ang = 1e-2;       % std. deviation for rotational noise, typical 1e-2
  sigma_cart = 1e-1;      % std. deviation for translational noise, typical 1e-1
  sigma_accel = 1e-3;     % std. deviation for accelerometer noise, typical 1e-3
  sigma_gyro = 1e-5;      % std. deviation for gyroscope noise, typical 1e-5
  sigma_accelBias = 1e-4; % std. deviation for added accelerometer constant bias, typical 1e-3
  sigma_gyroBias = 1e-6;  % std. deviation for added gyroscope constant bias, typical 1e-5
  sigma_gps = 1e-4;       % std. deviation for noise in GPS position measurements, typical 1e-4
  sigma_camera = 1;  % std. deviation for noise in camera measurements (pixels)
  
  % Set log files
  testName = sprintf('sa-%1.2g-sc-%1.2g-sacc-%1.2g-sg-%1.2g',sigma_ang,sigma_cart,sigma_accel,sigma_gyro)
  folderName = 'results/'
else
  fprintf('Tests have been externally configured.\n');
end

%% Between metadata
noiseVectorPose = [sigma_ang * ones(3,1); sigma_cart * ones(3,1)];
noisePose = noiseModel.Diagonal.Sigmas(noiseVectorPose);

%% Imu metadata
metadata.imu.epsBias = 1e-10; % was 1e-7
metadata.imu.g = [0;0;0];
metadata.imu.omegaCoriolis = [0;0;0];
metadata.imu.IntegrationSigma = 1e-5;
metadata.imu.zeroBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
metadata.imu.AccelerometerSigma = sigma_accel;
metadata.imu.GyroscopeSigma = sigma_gyro;
metadata.imu.BiasAccelerometerSigma = metadata.imu.epsBias;  % noise on expected change in accelerometer bias over time
metadata.imu.BiasGyroscopeSigma = metadata.imu.epsBias;      % noise on expected change in gyroscope bias over time
% noise on initial accelerometer and gyroscope biases
if options.imuNonzeroBias == 1
  metadata.imu.BiasAccOmegaInit = [sigma_accelBias * ones(3,1); sigma_gyroBias * ones(3,1)];
else
  metadata.imu.BiasAccOmegaInit = metadata.imu.epsBias * ones(6,1);
end

noiseVel =  noiseModel.Isotropic.Sigma(3, 1e-2); % was 0.1
noiseBiasBetween = noiseModel.Diagonal.Sigmas([metadata.imu.BiasAccelerometerSigma * ones(3,1);...
                                               metadata.imu.BiasGyroscopeSigma * ones(3,1)]); % between on biases
noisePriorBias = noiseModel.Diagonal.Sigmas(metadata.imu.BiasAccOmegaInit);

noiseVectorAccel = metadata.imu.AccelerometerSigma * ones(3,1);
noiseVectorGyro = metadata.imu.GyroscopeSigma  * ones(3,1);

%% GPS metadata
noiseVectorGPS = sigma_gps * ones(3,1);
noiseGPS = noiseModel.Diagonal.Precisions([zeros(3,1); 1/sigma_gps^2 * ones(3,1)]);

%% Camera metadata
metadata.camera.calibration = Cal3_S2(500,500,0,1920/2,1200/2); % Camera calibration
metadata.camera.xlims = [-100, 650];    % x limits on area for landmark creation
metadata.camera.ylims = [-100, 700];    % y limits on area for landmark creation
metadata.camera.zlims = [-30, 30];      % z limits on area for landmark creation
metadata.camera.visualRange = 100;      % maximum distance from the camera that a landmark can be seen (meters)
metadata.camera.bodyPoseCamera = Pose3; % pose of camera in body
metadata.camera.CameraSigma = sigma_camera;
cameraMeasurementNoise = noiseModel.Isotropic.Sigma(2, metadata.camera.CameraSigma);
noiseVectorCamera = metadata.camera.CameraSigma .* ones(2,1);

% Create landmarks and smart factors
if options.includeCameraFactors == 1
  for i = 1:options.numberOfLandmarks
    metadata.camera.gtLandmarkPoints(i) = Point3( ...
      [rand() * (metadata.camera.xlims(2)-metadata.camera.xlims(1)) + metadata.camera.xlims(1); ...  
       rand() * (metadata.camera.ylims(2)-metadata.camera.ylims(1)) + metadata.camera.ylims(1); ...
       rand() * (metadata.camera.zlims(2)-metadata.camera.zlims(1)) + metadata.camera.zlims(1)]);
  end
end


%% Create ground truth trajectory and measurements
[gtValues, gtMeasurements] = imuSimulator.covarianceAnalysisCreateTrajectory(options, metadata);

%% Create ground truth graph
% Set up noise models
gtNoiseModels.noisePose = noisePose;
gtNoiseModels.noiseVel = noiseVel;
gtNoiseModels.noiseBiasBetween = noiseBiasBetween;
gtNoiseModels.noisePriorPose = noisePose;
gtNoiseModels.noisePriorBias = noisePriorBias;
gtNoiseModels.noiseGPS = noiseGPS;
gtNoiseModels.noiseCamera = cameraMeasurementNoise;

% Set measurement noise to 0, because this is ground truth
gtMeasurementNoise.poseNoiseVector = zeros(6,1);
gtMeasurementNoise.imu.accelNoiseVector = zeros(3,1);
gtMeasurementNoise.imu.gyroNoiseVector = zeros(3,1);
gtMeasurementNoise.cameraNoiseVector = zeros(2,1);
gtMeasurementNoise.gpsNoiseVector = zeros(3,1);
  
% Set IMU biases to zero
metadata.imu.accelConstantBiasVector = zeros(3,1);
metadata.imu.gyroConstantBiasVector = zeros(3,1);
    
[gtGraph, projectionFactorSeenBy] = imuSimulator.covarianceAnalysisCreateFactorGraph( ...
    gtMeasurements, ...     % ground truth measurements
    gtValues, ...           % ground truth Values
    gtNoiseModels, ...      % noise models to use in this graph
    gtMeasurementNoise, ... % noise to apply to measurements
    options, ...            % options for the graph (e.g. which factors to include)
    metadata);              % misc data necessary for factor creation

%% Display, printing, and plotting of ground truth
%gtGraph.print(sprintf('\nGround Truth Factor graph:\n'));
%gtValues.print(sprintf('\nGround Truth Values:\n  '));

figure(1)
hold on;

if options.includeCameraFactors
  b = [-1000 2000 -2000 2000 -30 30];
  for i = 1:size(metadata.camera.gtLandmarkPoints,2)
      p = metadata.camera.gtLandmarkPoints(i);
      if(p(1) > b(1) && p(1) < b(2) && p(2) > b(3) && p(2) < b(4) && p(3) > b(5) && p(3) < b(6))
          plot3(p(1), p(2), p(3), 'k+');
      end
  end
  pointsToPlot = metadata.camera.gtLandmarkPoints(find(projectionFactorSeenBy > 0));
  for i = 1:length(pointsToPlot)
      p = pointsToPlot(i);
      plot3(p(1), p(2), p(3), 'gs', 'MarkerSize', 10);
  end
end
plot3DPoints(gtValues);
%plot3DTrajectory(gtValues, '-r', [], 1, Marginals(gtGraph, gtValues));
plot3DTrajectory(gtValues, '-r');

axis equal

% optimize
optimizer = GaussNewtonOptimizer(gtGraph, gtValues);
gtEstimate = optimizer.optimize();
plot3DTrajectory(gtEstimate, '-k');
% estimate should match gtValues if graph is correct.
fprintf('Error in ground truth graph at gtValues: %g \n', gtGraph.error(gtValues) );
fprintf('Error in ground truth graph at gtEstimate: %g \n', gtGraph.error(gtEstimate) );

disp('Plotted ground truth')

%% Monte Carlo Runs

% Set up noise models
monteCarloNoiseModels.noisePose = noisePose;
monteCarloNoiseModels.noiseVel = noiseVel;
monteCarloNoiseModels.noiseBiasBetween = noiseBiasBetween;
monteCarloNoiseModels.noisePriorPose = noisePose;
monteCarloNoiseModels.noisePriorBias = noisePriorBias;
monteCarloNoiseModels.noiseGPS = noiseGPS;
monteCarloNoiseModels.noiseCamera = cameraMeasurementNoise;

% Set measurement noise for monte carlo runs
monteCarloMeasurementNoise.poseNoiseVector = zeros(6,1); %noiseVectorPose;
monteCarloMeasurementNoise.imu.accelNoiseVector = noiseVectorAccel;
monteCarloMeasurementNoise.imu.gyroNoiseVector = noiseVectorGyro;
monteCarloMeasurementNoise.gpsNoiseVector = noiseVectorGPS;
monteCarloMeasurementNoise.cameraNoiseVector = noiseVectorCamera;
  
for k=1:numMonteCarloRuns
  fprintf('Monte Carlo Run %d...\n', k');

  % Create a random bias for each run
  if options.imuNonzeroBias == 1
    metadata.imu.accelConstantBiasVector = metadata.imu.BiasAccOmegaInit(1:3) .* randn(3,1);
    metadata.imu.gyroConstantBiasVector = metadata.imu.BiasAccOmegaInit(4:6) .* randn(3,1);
    %metadata.imu.accelConstantBiasVector = 1e-2 * ones(3,1);
    %metadata.imu.gyroConstantBiasVector = 1e-3 * ones(3,1);
  else
    metadata.imu.accelConstantBiasVector = zeros(3,1);
    metadata.imu.gyroConstantBiasVector = zeros(3,1);
  end
  
  % Create a new graph using noisy measurements
  [graph, projectionFactorSeenBy] = imuSimulator.covarianceAnalysisCreateFactorGraph( ...
    gtMeasurements, ...     % ground truth measurements
    gtValues, ...           % ground truth Values
    monteCarloNoiseModels, ...      % noise models to use in this graph
    monteCarloMeasurementNoise, ... % noise to apply to measurements
    options, ...            % options for the graph (e.g. which factors to include)
    metadata);              % misc data necessary for factor creation
      
  %graph.print('graph')
  
  % optimize
  optimizer = GaussNewtonOptimizer(graph, gtValues);
  estimate = optimizer.optimize();
  figure(1)
  plot3DTrajectory(estimate, '-b');
  
  marginals = Marginals(graph, estimate);
  
  % for each pose in the trajectory
  for i=0:options.trajectoryLength
    % compute estimation errors
    currentPoseKey = symbol('x', i);
    gtPosition  = gtValues.atPose3(currentPoseKey).translation;
    estPosition = estimate.atPose3(currentPoseKey).translation;
    estR = estimate.atPose3(currentPoseKey).rotation.matrix;
    errPosition = estPosition - gtPosition;
    
    % compute covariances:
    cov = marginals.marginalCovariance(currentPoseKey);
    covPosition = estR * cov(4:6,4:6) * estR';
    % compute NEES using (estimationError = estimatedValues - gtValues) and estimated covariances
    NEES(k,i+1) = errPosition' * inv(covPosition) * errPosition; % distributed according to a Chi square with n = 3 dof
  end
  
  figure(2)
  hold on
  plot(NEES(k,:),'-b','LineWidth',1.5)
end
%%
ANEES = mean(NEES);
plot(ANEES,'-r','LineWidth',2)
plot(3*ones(size(ANEES,2),1),'k--'); % Expectation(ANEES) = number of dof
box on
set(gca,'Fontsize',16)
title('NEES and ANEES');
if saveResults
  saveas(gcf,horzcat(folderName,'runs-',testName,'.fig'),'fig');
  saveas(gcf,horzcat(folderName,'runs-',testName,'.png'),'png');
end

%%
figure(1)
box on
set(gca,'Fontsize',16)
title('Ground truth and estimates for each MC runs');
if saveResults
  saveas(gcf,horzcat(folderName,'gt-',testName,'.fig'),'fig');
  saveas(gcf,horzcat(folderName,'gt-',testName,'.png'),'png');
end

%% Let us compute statistics on the overall NEES
n = 3; % position vector dimension
N = numMonteCarloRuns; % number of runs
alpha = 0.01; % confidence level

% mean_value = n*N; % mean value of the Chi-square distribution
% (we divide by n * N and for this reason we expect ANEES around 1)
r1 = chi2inv(alpha, n * N)  / (n * N);
r2 = chi2inv(1-alpha, n * N)  / (n * N);

% output here
fprintf(1, 'r1 = %g\n', r1);
fprintf(1, 'r2 = %g\n', r2);

figure(3)
hold on
plot(ANEES/n,'-b','LineWidth',2)
plot(ones(size(ANEES,2),1),'r-');
plot(r1*ones(size(ANEES,2),1),'k-.');
plot(r2*ones(size(ANEES,2),1),'k-.');
box on
set(gca,'Fontsize',16)
title('NEES normalized by dof VS bounds');
if saveResults
  saveas(gcf,horzcat(folderName,'ANEES-',testName,'.fig'),'fig');
  saveas(gcf,horzcat(folderName,'ANEES-',testName,'.png'),'png');
  logFile = horzcat(folderName,'log-',testName);
  save(logFile)
end

%% NEES COMPUTATION (Bar-Shalom 2001, Section 5.4)
% the nees for a single experiment (i) is defined as
%               NEES_i = xtilda' * inv(P) * xtilda,
% where xtilda in R^n is the estimation
% error, and P is the covariance estimated by the approach we want to test
%
% Average NEES. Given N Monte Carlo simulations, i=1,...,N, the average
% NEES is:
%                   ANEES = sum(NEES_i)/N
% The quantity N*ANEES is distributed according to a Chi-square
% distribution with N*n degrees of freedom.
%
% For the single run case, N=1, therefore NEES = ANEES is distributed
% according to a chi-square distribution with n degrees of freedom (e.g. n=3
% if we are testing a position estimate)
% Therefore its mean should be n (difficult to see from a single run)
% and, with probability alpha, it should hold:
%
% NEES in [r1, r2]
%
% where r1 and r2 are built from the Chi-square distribution

