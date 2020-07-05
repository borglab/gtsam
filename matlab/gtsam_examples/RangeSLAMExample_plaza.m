%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read Robotics Institute range-only Plaza2 dataset and do SAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% preliminaries
clear
import gtsam.*

%% Find and load data file
% data available at http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/
% Datafile format (from http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/log.html)
% GT: Groundtruth path from GPS
%    Time (sec)	X_pose (m)	Y_pose (m)	Heading (rad)
% DR: Odometry Input (delta distance traveled and delta heading change)
%    Time (sec)	Delta Dist. Trav. (m)	Delta Heading (rad)
% DRp: Dead Reckoned Path from Odometry
%    Time (sec)	X_pose (m)	Y_pose (m)	Heading (rad)
% TL: Surveyed Node Locations
%    Time (sec)	X_pose (m)	Y_pose (m)
% TD
%    Time (sec)	Sender / Antenna ID	Receiver Node ID	Range (m)
datafile = findExampleDataFile('Plaza2_.mat');
load(datafile)
M=size(DR,1);
K=size(TD,1);
sigmaR = 50; % range standard deviation
sigmaInitial = 1; % draw initial landmark guess from Gaussian

%% Set Noise parameters
noiseModels.prior = noiseModel.Diagonal.Sigmas([1 1 pi]');
noiseModels.pointPrior = noiseModel.Diagonal.Sigmas([1 1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.01 0.2]');
base = noiseModel.mEstimator.Tukey(5);
noiseModels.range = noiseModel.Robust(base,noiseModel.Isotropic.Sigma(1, sigmaR));

%% Add prior on first pose
pose0 = Pose2(GT(1,2),GT(1,3),pi+GT(1,4));
graph = NonlinearFactorGraph;
graph.add(PriorFactorPose2(0,pose0,noiseModels.prior));
initial = Values;
initial.insert(0,pose0);

for i=1:size(TL,1)
  j=TL(i,1);
  initial.insert(symbol('L',j),Point2(sigmaInitial*randn,sigmaInitial*randn));
end

%% Loop over odometry
tic
k = 1; % range measurement counter
lastPose = pose0;
for i=1:M
  
  % get odometry measurement
  t = DR(i,1);
  distance_traveled = DR(i,2);
  delta_heading = DR(i,3);
  
  % add odometry factor
  odometry = Pose2(distance_traveled,0,delta_heading);
  graph.add(BetweenFactorPose2(i-1, i, odometry, noiseModels.odometry));
  
  % predict pose and add as initial estimate
  predictedPose = lastPose.compose(odometry);
  lastPose = predictedPose;
  initial.insert(i,predictedPose);
  
  while k<=K && t>=TD(k,1)
    j = TD(k,3);
    range = TD(k,4);
    factor = RangeFactor2D(i, symbol('L',j), range, noiseModels.range);
    graph.add(factor);
    k=k+1;
  end
  
end
toc

%% Graph was built, optimize !
tic
batchOptimizer = LevenbergMarquardtOptimizer(graph, initial);
result = batchOptimizer.optimize();
toc

%% visualize
figure(1);clf;hold on

% odometry
XYT = utilities.extractPose2(initial);
plot(XYT(:,1),XYT(:,2),'y-');

% GT
plot(GT(:,2),GT(:,3),'g-');
plot(TL(:,2),TL(:,3),'g*');

% result
XYT = utilities.extractPose2(result);
plot(XYT(:,1),XYT(:,2),'k-');
XY = utilities.extractPoint2(result);
plot(XY(:,1),XY(:,2),'k*');
axis equal
