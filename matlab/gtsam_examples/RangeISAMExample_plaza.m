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
datafile = findExampleDataFile('Plaza2_.mat');
load(datafile)
M=size(DR,1);
K=size(TD,1);
minK=100; % minimum number of range measurements to process initially
incK=5; % minimum number of range measurements to process after
sigmaR = 100; % range standard deviation
sigmaInitial = 1; % draw initial landmark guess from Gaussian
useGroundTruth = false;
useRobust=true;
addRange=true;
useResult=true;
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
params = gtsam.ISAM2Params;
params.setRelinearizeSkip(1);
% gnParams = ISAM2GaussNewtonParams;
% dlParams = ISAM2DoglegParams;
% params.setOptimizationParams(gnParams);
isam = ISAM2(params);

%% Add prior on first pose
pose0 = Pose2(GT(1,2),GT(1,3),pi+GT(1,4));
newFactors = NonlinearFactorGraph;
if ~addRange | ~useGroundTruth
  newFactors.add(PriorFactorPose2(0,pose0,noiseModels.prior));
end
initial = Values;
initial.insert(0,pose0);
odo = Values;
odo.insert(0,pose0);

if addRange
  for i=1:size(TL,1)
    j=TL(i,1);
    if useGroundTruth
      Lj = Point2(TL(i,2),TL(i,3));
      initial.insert(symbol('L',j),Lj);
      newFactors.add(PriorFactorPoint2(symbol('L',j),Lj,noiseModels.pointPrior));
    else
      initial.insert(symbol('L',j),Point2(sigmaInitial*randn,sigmaInitial*randn));
    end
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
  
  while k<=K & t>=TD(k,1)
    j = TD(k,3);
    range = TD(k,4);
    %     fprintf(1,'%7g %7g %5d %5d %d %0.2f\n', t,TD(k,1),k, i, j, range);
    if addRange
      newFactors.add(RangeFactorPosePoint2(i, symbol('L',j), range, noiseModels.range));
    end
    k=k+1; countK=countK+1; update = true;
  end
  
  if update & k>minK & countK>incK
    if batchInitialization % Do a full optimize for first minK ranges
      tic
      batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initial);
      toc
      initial = batchOptimizer.optimize();
      batchInitialization = false; % only once
    end
    isam.update(newFactors, initial);
    if useResult
      result = isam.calculateEstimate(); 
      lastPose = result.at(i);
    else
      lin = isam.getLinearizationPoint();
      lastPose = lin.at(i);
    end
    newFactors = NonlinearFactorGraph;
    initial = Values;
    countK = 0;
  end
  
  % visualize
  if mod(i,50)==0 & k>minK
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
