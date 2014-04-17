import gtsam.*;

% Test GTSAM covariances on a graph with betweenFactors
% Optionally, you can also enable IMU factors and Camera factors
% Authors: Luca Carlone, David Jensen
% Date: 2014/4/6

clc
clear all
close all

%% Configuration
options.useRealData = 1;           % controls whether or not to use the real data (if available) as the ground truth traj
options.includeBetweenFactors = 1; % if true, BetweenFactors will be generated between consecutive poses
options.includeIMUFactors = 1;     % if true, IMU factors will be generated for the trajectory based on options.imuFactorType
options.imuFactorType = 1;         % Set to 1 or 2 to use IMU type 1 or type 2 factors (will default to type 1)
options.includeCameraFactors = 0;  % not fully implemented yet
options.trajectoryLength = 209;    % length of the ground truth trajectory
options.subsampleStep = 20;        % number of poses to skip when using real data (to reduce computation on long trajectories)

numMonteCarloRuns = 2;             % number of Monte Carlo runs to perform

%% Camera metadata
numberOfLandmarks = 10;    % Total number of visual landmarks, used for camera factors
K = Cal3_S2(500,500,0,640/2,480/2); % Camera calibration
cameraMeasurementNoiseSigma = 1.0;
cameraMeasurementNoise = noiseModel.Isotropic.Sigma(2,cameraMeasurementNoiseSigma);

% Create landmarks
if options.includeCameraFactors == 1
  for i = 1:numberOfLandmarks
    gtLandmarkPoints(i) = Point3( ...
      ... % uniformly distributed in the x axis along 120% of the trajectory length, starting after 15 poses
      [rand()*20*(options.trajectoryLength*1.2) + 15*20; ...  
      randn()*20; ...   % normally distributed in the y axis with a sigma of 20
      randn()*20]);     % normally distributed in the z axis with a sigma of 20
  end
end

%% Imu metadata
metadata.imu.epsBias = 1e-10; % was 1e-7
metadata.imu.zeroBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
metadata.imu.AccelerometerSigma = 1e-5;
metadata.imu.GyroscopeSigma = 1e-7;
metadata.imu.IntegrationSigma = 1e-4;
metadata.imu.BiasAccelerometerSigma = metadata.imu.epsBias;
metadata.imu.BiasGyroscopeSigma = metadata.imu.epsBias;
metadata.imu.BiasAccOmegaInit = metadata.imu.epsBias;
metadata.imu.g = [0;0;0];
metadata.imu.omegaCoriolis = [0;0;0];
noiseVel =  noiseModel.Isotropic.Sigma(3, 1e-2); % was 0.1
noiseBias = noiseModel.Isotropic.Sigma(6, metadata.imu.epsBias);
noisePriorBias = noiseModel.Isotropic.Sigma(6, 1e-4);

sigma_accel = metadata.imu.AccelerometerSigma;
sigma_gyro = metadata.imu.GyroscopeSigma;
noiseVectorAccel = [sigma_accel; sigma_accel; sigma_accel];
noiseVectorGyro = [sigma_gyro; sigma_gyro; sigma_gyro];


%% Between metadata
sigma_ang = 1e-3;  sigma_cart = 1e-3;
noiseVectorPose = [sigma_ang; sigma_ang; sigma_ang; sigma_cart; sigma_cart; sigma_cart];
noisePose = noiseModel.Diagonal.Sigmas(noiseVectorPose);
%noisePose = noiseModel.Isotropic.Sigma(6, 1e-3);

%% Set log files
testName = sprintf('sa-%1.2g-sc-%1.2g',sigma_ang,sigma_cart)
folderName = 'results/'

%% Create ground truth trajectory and measurements
[gtValues, gtMeasurements] = imuSimulator.covarianceAnalysisCreateTrajectory(options, metadata);

%% Create ground truth graph
% Set up noise models
gtNoiseModels.noisePose = noisePose;
gtNoiseModels.noiseVel = noiseVel;
gtNoiseModels.noiseBias = noiseBias;
gtNoiseModels.noisePriorPose = noisePose;
gtNoiseModels.noisePriorBias = noisePriorBias;

% Set measurement noise to 0, because this is ground truth
gtMeasurementNoise.poseNoiseVector = [0; 0; 0; 0; 0; 0;];
gtMeasurementNoise.imu.accelNoiseVector = [0; 0; 0];
gtMeasurementNoise.imu.gyroNoiseVector = [0; 0; 0];
gtMeasurementNoise.cameraPixelNoiseVector = [0; 0];
  
gtGraph = imuSimulator.covarianceAnalysisCreateFactorGraph( ...
    gtMeasurements, ...     % ground truth measurements
    gtValues, ...           % ground truth Values
    gtNoiseModels, ...      % noise models to use in this graph
    gtMeasurementNoise, ... % noise to apply to measurements
    options, ...            % options for the graph (e.g. which factors to include)
    metadata);              % misc data necessary for factor creation

%% Display, printing, and plotting of ground truth
%gtGraph.print(sprintf('\nGround Truth Factor graph:\n'));
%gtValues.print(sprintf('\nGround Truth Values:\n  '));

warning('Additional prior on zerobias')
warning('Additional PriorFactorLieVector on velocities')

figure(1)
hold on;
plot3DPoints(gtValues);
plot3DTrajectory(gtValues, '-r', [], 1, Marginals(gtGraph, gtValues));
axis equal

disp('Plotted ground truth')

%% Monte Carlo Runs

% Set up noise models
monteCarloNoiseModels.noisePose = noisePose;
monteCarloNoiseModels.noiseVel = noiseVel;
monteCarloNoiseModels.noiseBias = noiseBias;
monteCarloNoiseModels.noisePriorPose = noisePose;
monteCarloNoiseModels.noisePriorBias = noisePriorBias;

% Set measurement noise for monte carlo runs
monteCarloMeasurementNoise.poseNoiseVector = noiseVectorPose;
monteCarloMeasurementNoise.imu.accelNoiseVector = noiseVectorAccel;
monteCarloMeasurementNoise.imu.gyroNoiseVector = noiseVectorGyro;
monteCarloMeasurementNoise.cameraPixelNoiseVector = [0; 0];
  
for k=1:numMonteCarloRuns
  fprintf('Monte Carlo Run %d.\n', k');

  % Create a new graph using noisy measurements
  graph = imuSimulator.covarianceAnalysisCreateFactorGraph( ...
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
  for i=1:size(gtMeasurements.deltaMatrix,1)+1
    % compute estimation errors
    currentPoseKey = symbol('x', i-1);
    gtPosition  = gtValues.at(currentPoseKey).translation.vector;
    estPosition = estimate.at(currentPoseKey).translation.vector;
    estR = estimate.at(currentPoseKey).rotation.matrix;
    errPosition = estPosition - gtPosition;
    
    % compute covariances:
    cov = marginals.marginalCovariance(currentPoseKey);
    covPosition = estR * cov(4:6,4:6) * estR';
    
    % compute NEES using (estimationError = estimatedValues - gtValues) and estimated covariances
    NEES(k,i) = errPosition' * inv(covPosition) * errPosition; % distributed according to a Chi square with n = 3 dof
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
%print('-djpeg', horzcat('runs-',testName));
saveas(gcf,horzcat(folderName,'runs-',testName,'.fig'),'fig');

%%
figure(1)
box on
set(gca,'Fontsize',16)
title('Ground truth and estimates for each MC runs');
%print('-djpeg', horzcat('gt-',testName));
saveas(gcf,horzcat(folderName,'gt-',testName,'.fig'),'fig');

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
%print('-djpeg', horzcat('ANEES-',testName));
saveas(gcf,horzcat(folderName,'ANEES-',testName,'.fig'),'fig');

logFile = horzcat(folderName,'log-',testName);
save(logFile)

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

