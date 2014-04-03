% Test GTSAM covariances on a graph with betweenFactors

clc
clear all
close all

%% Create ground truth trajectory
trajectoryLength = 100;

% possibly create random trajectory
currentPoseKey = symbol('x', 0);
currentPose = Pose3;
gtValues = Values;
gtValues.insert(currentPoseKey, currentPose);

for i=1:trajectoryLength
  currentPoseKey = symbol('x', i);
  deltaPosition = % create random vector with mean [x 0 0]
  deltaRotation = % create random rotation with mean [0 0 0]
  deltaPose = Pose3(deltaRotation, Point3(deltaPosition));
  % "Deduce" ground truth measurements
  % deltaPose are the gt measurements - save them in some structure
  currentPose = currentPose.compose(deltaPose);
  gtValues.insert(currentPoseKey, currentPose);
end

%% Create gt graph (using between with ground truth measurements)
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


