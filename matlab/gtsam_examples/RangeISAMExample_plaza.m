%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read Robotics Institute range-only Plaza2 dataset and do iSAM
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
if true % switch between data files
  datafile = findExampleDataFile('Plaza1_.mat');
  headingOffset=0;
  minK=200; % minimum number of range measurements to process initially
  incK=5; % minimum number of range measurements to process after
else
  datafile = findExampleDataFile('Plaza2_.mat');
  headingOffset=pi;
  minK=150; % needs less for init
  incK=25; % minimum number of range measurements to process after
end
load(datafile)
M=size(DR,1);
K=size(TD,1);
sigmaR = 100; % range standard deviation
sigmaInitial = 1; % draw initial landmark guess from Gaussian
useGroundTruth = false;
useRobust=true;
addRange=true;
batchInitialization=true;

%% Set Noise parameters
noiseModels.prior = noiseModel.Diagonal.Sigmas([1 1 pi]');
noiseModels.pointPrior = noiseModel.Diagonal.Sigmas([1 1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.01 0.2]');
if useRobust
  base = noiseModel.mEstimator.Tukey(15);
  noiseModels.range = noiseModel.Robust(base,noiseModel.Isotropic.Sigma(1, sigmaR));
else
  noiseModels.range = noiseModel.Isotropic.Sigma(1, sigmaR);
end

%% Initialize iSAM
isam = ISAM2;

%% Add prior on first pose
pose0 = Pose2(GT(1,2),GT(1,3),headingOffset+GT(1,4));
newFactors = NonlinearFactorGraph;
if ~addRange || ~useGroundTruth
  newFactors.add(PriorFactorPose2(0,pose0,noiseModels.prior));
end
initial = Values;
initial.insert(0,pose0);
odo = Values;
odo.insert(0,pose0);

%% initialize points
if addRange
  landmarkEstimates = Values;
  for i=1:size(TL,1)
    j=TL(i,1);
    if useGroundTruth
      Lj = Point2(TL(i,2),TL(i,3));
      newFactors.add(PriorFactorPoint2(symbol('L',j),Lj,noiseModels.pointPrior));
    else
      Lj = Point2(sigmaInitial*randn,sigmaInitial*randn);
    end
    initial.insert(symbol('L',j),Lj);
    landmarkEstimates.insert(symbol('L',j),Lj);
  end
  XY = utilities.extractPoint2(initial);
  plot(XY(:,1),XY(:,2),'g*');
end

%% Loop over odometry
tic
k = 1; % range measurement counter
update = false;
lastPose = pose0;
odoPose = pose0;
countK = 0;
for i=1:M % M
  
  % get odometry measurement
  t = DR(i,1);
  distance_traveled = DR(i,2);
  delta_heading = DR(i,3);
  
  % add odometry factor
  odometry = Pose2(distance_traveled,0,delta_heading);
  newFactors.add(BetweenFactorPose2(i-1, i, odometry, noiseModels.odometry));
  
  % predict pose and update odometry
  predictedOdo = odoPose.compose(odometry);
  odoPose = predictedOdo;
  odo.insert(i,predictedOdo);
  
  % predict pose and add as initial estimate
  predictedPose = lastPose.compose(odometry);
  lastPose = predictedPose;
  initial.insert(i,predictedPose);
  landmarkEstimates.insert(i,predictedPose);
  
  % Check if there are range factors to be added
  while k<=K && t>=TD(k,1)
    j = TD(k,3);
    range = TD(k,4);
    if addRange
      factor = RangeFactor2D(i, symbol('L',j), range, noiseModels.range);
      % Throw out obvious outliers based on current landmark estimates
      error=factor.unwhitenedError(landmarkEstimates);
      if k<=minK || abs(error)<5
        newFactors.add(factor);
      end
    end
    k=k+1; countK=countK+1; update = true;
  end

  % Check whether to update iSAM 2
  if update && k>minK && countK>incK
    if batchInitialization % Do a full optimize for first minK ranges
      tic
      batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initial);
      initial = batchOptimizer.optimize();
      toc
      batchInitialization = false; % only once
    end
    isam.update(newFactors, initial);
    result = isam.calculateEstimate();
    lastPose = result.atPose2(i);
    % update landmark estimates
    if addRange
      landmarkEstimates = Values;
      for jj=1:size(TL,1)
        j=TL(jj,1);
        key = symbol('L',j);
        landmarkEstimates.insert(key,result.atPoint2(key));
      end
    end
    newFactors = NonlinearFactorGraph;
    initial = Values;
    countK = 0;
  end
  
  % visualize
  if mod(i,50)==0 && k>minK
    figure(1);clf;hold on
    
    % odometry
    XYT = utilities.extractPose2(odo);
    plot(XYT(:,1),XYT(:,2),'y-');
    
    % lin point
    lin = isam.getLinearizationPoint();
    XYT = utilities.extractPose2(lin);
    plot(XYT(:,1),XYT(:,2),'r.');
    XY = utilities.extractPoint2(lin);
    plot(XY(:,1),XY(:,2),'r*');
    
    % result
    result = isam.calculateEstimate();
    XYT = utilities.extractPose2(result);
    plot(XYT(:,1),XYT(:,2),'k-');
    XY = utilities.extractPoint2(result);
    plot(XY(:,1),XY(:,2),'k*');
    axis equal
    %     pause
  end
end
toc

%% Plot ground truth as well
plot(GT(:,2),GT(:,3),'g-');
plot(TL(:,2),TL(:,3),'g*');


