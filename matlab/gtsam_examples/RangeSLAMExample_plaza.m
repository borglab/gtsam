%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2014, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read Robotics Institute range-only Plaza2 dataset and do SAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Range-only SLAM
% This example demonstrates how to use GTSAM for range-only SLAM. We will
% read the data from a file, and then build a factor graph containing a
% RangeFactorPosePoint2 for every range measurement, a BetweenFactorPose2 for 
% every robot odometry measurement, a as well as a prior on the first pose.
% This is a batch-SLAM approach, an incremental SLAM approach might be less
% prone to local minima, but overall it works quite well and quite fast.
%
% We start by clearning the workspace and importing all of GTSAM:
clear
import gtsam.*

%% Load Data
% The data is available at http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/
% The datafile format (from http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/log.html)
%
% * GT: Groundtruth path from GPS: 
%       |Time (sec) X_pose (m) Y_pose (m) Heading (rad)|
% * DR: Odometry Input (delta distance traveled and delta heading change): 
%       |Time (sec) Delta Dist. Trav. (m) Delta Heading (rad)|
% * DRp: Dead Reckoned Path from Odometry: 
%       |Time (sec) X_pose (m) Y_pose (m) Heading (rad)|
% * TL: Surveyed Node Locations: 
%       |Time (sec) X_pose (m) Y_pose (m)|
% * TD: Range measurements:  
%       |Time (sec) Sender/Antenna ID Receiver Node (ID) Range (m)|
%
% Example data can be found through the gtsam.findExampleDataFile utility:
datafile = findExampleDataFile('Plaza2_.mat');
load(datafile)
M=size(DR,1); % number of odometry measurements
K=size(TD,1); % number of range measurements

%% Create Factor Graph and Values
% We create a factor graph that holds nonlinear factors, which will then be
% repeatedly linearized during optimization. The factor graph defines a
% probability distribution, but does not contain any values for the unknown
% variables. Thoese will be initialized in the Values object 'initial'.
graph = NonlinearFactorGraph;
initial = Values;

%% Create Noise Models
% Below we create the GTSAM noise models that are needed for factor
% creation. All of them are Gaussian noise models with a diagnoal
% covariance matrix (although we specifiy them below in terms of standard
% deviations), but for the range we use a robust Tukey error model.
noiseModels.prior = noiseModel.Diagonal.Sigmas([1 1 pi]');
noiseModels.pointPrior = noiseModel.Diagonal.Sigmas([1 1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.01 0.2]');
base = noiseModel.mEstimator.Tukey(5);
sigmaR = 50; % range standard deviation
noiseModels.range = noiseModel.Robust(base,noiseModel.Isotropic.Sigma(1, sigmaR));

%% Prior on First Pose
% We extract the first pose from the ground truth, add this as a prior on 
% the first pose (with key 0), and also initialize the unknown with it.
pose0 = Pose2(GT(1,2),GT(1,3),pi+GT(1,4));
graph.add(PriorFactorPose2(0,pose0,noiseModels.prior));
initial.insert(0,pose0);

%% Landmark initialization
% We randomly initialize the landmarks. We could get unlucky with this and
% get stuck in a local minimum. A smarter strategy would initialize them by
% triangulation after seeing a few range measurements.
sigmaInitial = 30; % draw initial landmark guess from Gaussian
for i=1:size(TL,1)
  j=TL(i,1);
  initial.insert(symbol('L',j),Point2(pose0.x+sigmaInitial*randn,pose0.y+sigmaInitial*randn));
end

%% Add Measurement Factors
% Below we loop over the odometry measurements, and for each of those add
% an odometry factor (a BetweenFactorPose2) and insert an initial guess for
% the corresponding next pose. After that we scan the TD matrix for range
% measurements, and add a range factor for each. The type of this factor is a 
% RangeFactorPose2Point2, the equivalent of the C++ RangeFactor<Pose2,Point2>
% template instantiation. 
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
  
  % scan the TD matrix for range measurements
  while k<=K && t>=TD(k,1)
    j = TD(k,3);
    range = TD(k,4);
    factor = RangeFactorPosePoint2(i, symbol('L',j), range, noiseModels.range);
    graph.add(factor);
    k=k+1;
  end
  
end
toc

%% Optimization
% The factor graph is now built, so we can now optimize. Below we use the
% GTSAM Levenberg-Marqaurdt nonlinear optimizer, whch just takes a graph
% and a set of initial values. After that, we simply call optimize and wait.
tic
batchOptimizer = LevenbergMarquardtOptimizer(graph, initial);
result = batchOptimizer.optimize();
toc

%% Visualization of Results
% Below we show the initial estimate derived from the odometry in yellow,
% the ground truth in green, and the estimated trajectory in black.
figure(1);clf;hold on

% initial estimate
XYT = utilities.extractPose2(initial);
plot(XYT(:,1),XYT(:,2),'y-');

% ground truth
plot(GT(:,2),GT(:,3),'g-');
plot(TL(:,2),TL(:,3),'g*');

% estimated trajectory
XYT = utilities.extractPose2(result);
plot(XYT(:,1),XYT(:,2),'k-');
XY = utilities.extractPoint2(result);
plot(XY(:,1),XY(:,2),'k*');
axis equal
