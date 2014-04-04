import gtsam.*;

% Test GTSAM covariances on a graph with betweenFactors

clc
clear all
close all

%% Create ground truth trajectory
trajectoryLength = 5;

% possibly create random trajectory
currentPoseKey = symbol('x', 0);
currentPose = Pose3;
gtValues = Values;
gtValues.insert(currentPoseKey, currentPose);
gtGraph = NonlinearFactorGraph;
gtNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.05; 0.05; 0.05]); % Noise for GT measurements

for i=1:trajectoryLength
  currentPoseKey = symbol('x', i);
  deltaPosition = 0.5*rand(3,1) + [1;0;0]; % create random vector with mean = [1 0 0] and sigma = 0.5
  deltaRotation = 0.1*rand(3,1) + [0;0;0]; % create random rotation with mean [0 0 0] and sigma = 0.1 (rad)
  deltaPose = Pose3.Expmap([deltaRotation; deltaPosition]);
  deltaPoseNoise = gtNoise;
  
  % "Deduce" ground truth measurements
  % deltaPose are the gt measurements - save them in some structure
  gtMeasurementPose(i) = deltaPose;
  currentPose = currentPose.compose(deltaPose);
  gtValues.insert(currentPoseKey, currentPose);
  
  % Add the factor to the factor graph
  if(i == 1)
      gtGraph.add(PriorFactorPose3(currentPoseKey, deltaPose, deltaPoseNoise));
  else
      gtGraph.add(BetweenFactorPose3(previousPoseKey, currentPoseKey, deltaPose, deltaPoseNoise));
  end
  previousPoseKey = currentPoseKey;
end

gtGraph.print(sprintf('\nGround Truth - Factor graph:\n'));
gtValues.print(sprintf('\nGround Truth - Values:\n'));

%% Create gt graph (using between with ground truth measurements)
% Optimize using Levenberg-Marquardt
optimizer = LevenbergMarquardtOptimizer(gtGraph, gtValues);
gtResult = optimizer.optimizeSafely();
gtResult.print(sprintf('\nGround Truth - Final Result:\n'));

% Plot trajectory and covariance ellipses
% Couldn't get this to work in the modified example (OdometryExample3D).
% Something strange with 3D trajectories?
cla;
hold on;

plot3DTrajectory(gtResult, [], Marginals(gtGraph, gtResult));
axis equal

% Compute covariances using gtGraph and gtValues (for visualization)

% decide measurement covariance

%% for k=1:numMonteCarloRuns
% create a new graph
% for each measurement. add noise and add to graph
% optimize
% compute covariances: 
% compute NEES using (estimationError = estimatedValues - gtValues) and estimated covariances
% "estimationError = estimatedValues - gtValues" only holds in a linear case
% in a nonlinear case estimationError = LogMap ((estimatedValues.inverse) * gtValues)
% in GTSAM you should check "localCoordinates"

%% compute statistics: ANEES, plots


