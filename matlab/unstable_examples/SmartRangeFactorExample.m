clear all; clear all;
clc
close all
import gtsam.*;

%% Noise settings
t_noise = 0.1;      % odometry tranlation error
r_noise = 0.05;     % odometry rotation error 
range_noise = 0.001; % range measurements error

% Create noise models
noiseRange = noiseModel.Isotropic.Sigma(1, range_noise);  % range measurements noise model
noiseOdom = noiseModel.Diagonal.Sigmas([t_noise; t_noise; r_noise]); % odometry noise model
noiseInit = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.0001]); % for prior on first pose

%% Choose initial guess for landmarks
% if set to 1 we set the landmark guess to the true position
goodInitFlag_lmk1 = 0; 
goodInitFlag_lmk2 = 1;
goodInitFlag_lmk3 = 1;
% true positions
lmk1 = Point2([0.5;0.5]);
lmk2 = Point2([1.5;0.5]);
lmk3 = Point2([2.5;0.5]);
% specular positions (to simulate ambiguity in triangulation)
lmk1_bad = Point2([0.5;-0.5]);
lmk2_bad = Point2([1.5;1.5]);
lmk3_bad = Point2([2.5;-0.5]);

%% Create keys
num_poses = 7;
for ind_pose = 1:num_poses
  poseKey(ind_pose) = symbol('x',ind_pose); % Key for each pose
end
lmkKey(1) = symbol('l',1); % Key for each pose
lmkKey(2) = symbol('l',2); % Key for each pose
lmkKey(3) = symbol('l',3); % Key for each pose

%% Factor graphs
smartGraph = NonlinearFactorGraph;
smartValues = Values;

fullGraph = NonlinearFactorGraph;
fullValues = Values;

% Add prior on first pose 
poseInit = Pose2;
smartValues.insert(poseKey(1), poseInit );
smartGraph.add(PriorFactorPose2(poseKey(1), poseInit, noiseInit));
fullValues.insert(poseKey(1), poseInit );
fullGraph.add(PriorFactorPose2(poseKey(1), poseInit, noiseInit));
currPose = poseInit;

srf1 = SmartRangeFactor(range_noise);
srf2 = SmartRangeFactor(range_noise);
srf3 = SmartRangeFactor(range_noise);

%% Main loop
for ind_pose = 2:7
  ind_pose
   
  %% apply command, depending on the time step
  switch ind_pose
    case 2 
      v = [1; 0; pi/2];
    case 3 
      v = [1; 0; -pi/2];
    case 4 
      v = [1; 0; -pi/2];
    case 5 
      v = [1; 0; pi/2];
    case 6 
      v = [1; 0; pi/2];
    case 7 
      v = [1; 0; 0];
  end
  
  %% compute intial guess for poses (initialized to ground truth)
  currPose = currPose.retract(v);
  smartValues.insert(poseKey(ind_pose), currPose );
  fullValues.insert(poseKey(ind_pose), currPose );
  
  key_prev = poseKey(ind_pose-1);
  key_curr = poseKey(ind_pose);
  prev_pose = smartValues.atPose2(key_prev);
  curr_pose = smartValues.atPose2(key_curr);
  GTDeltaPose = prev_pose.between(curr_pose);
  noisePose = Pose2([t_noise*rand; t_noise*rand; r_noise*rand]);
  NoisyDeltaPose = GTDeltaPose.compose(noisePose);
  
  % add odometry factors
  smartGraph.add(BetweenFactorPose2(key_prev, key_curr, NoisyDeltaPose, noiseOdom));
  fullGraph.add(BetweenFactorPose2(key_prev, key_curr, NoisyDeltaPose, noiseOdom));
  
  % add range factors
  switch ind_pose
    %====================================================================
    case 2
      % x1-lmk1
      % x2-lmk1
      r1 = prev_pose.range(lmk1); % range of lmk1 wrt x1
      srf1.addRange(key_prev, r1);
      r2 = curr_pose.range(lmk1); % range of lmk1 wrt x2
      srf1.addRange(key_curr, r2);
      
      rangef1 = RangeFactor2D(key_prev, lmkKey(1), r1, noiseRange);
      fullGraph.add(rangef1); 
      rangef2 = RangeFactor2D(key_curr, lmkKey(1), r2, noiseRange);
      fullGraph.add(rangef2); 
      
      if goodInitFlag_lmk1==1
        fullValues.insert(lmkKey(1), lmk1);
      else
        fullValues.insert(lmkKey(1), lmk1_bad);
      end
    %====================================================================
    case 3
      % x3-lmk1
      % x3-lmk2
      r3 = curr_pose.range(lmk1); % range of lmk1 wrt x3
      srf1.addRange(key_curr, r3);
      smartGraph.add(srf1);
      r4 = curr_pose.range(lmk2); % range of lmk2 wrt x3
      srf2.addRange(key_curr, r4);    
      
      rangef3 = RangeFactor2D(key_curr, lmkKey(1), r3, noiseRange);
      fullGraph.add(rangef3); 
      rangef4 = RangeFactor2D(key_curr, lmkKey(2), r4, noiseRange);
      % IF WE ADD FACTOR HERE IT CRASHES: fullGraph.add(rangef4);  
    %====================================================================  
    case 4
      % x4-lmk2
      % x4-lmk3
      r5 = curr_pose.range(lmk2); % range of lmk2 wrt x4
      srf2.addRange(key_curr, r5);
      r6 = curr_pose.range(lmk3); % range of lmk3 wrt x4
      srf3.addRange(key_curr, r6);  
      
      % DELAYED INITIALIZATION: 
      fullGraph.add(rangef4); 
      rangef5 = RangeFactor2D(key_curr, lmkKey(2), r5, noiseRange);
      fullGraph.add(rangef5); 
      rangef6 = RangeFactor2D(key_curr, lmkKey(3), r6, noiseRange);
      % IF WE ADD FACTOR HERE IT CRASHES:  fullGraph.add(rangef6); 
      
      if goodInitFlag_lmk2==1
        fullValues.insert(lmkKey(2), lmk2);
      else
        fullValues.insert(lmkKey(2), lmk2_bad);
      end
    %====================================================================      
    case 5
      % x5-lmk2
      % x4-lmk3
      r7 = curr_pose.range(lmk2); % range of lmk2 wrt x5
      srf2.addRange(key_curr, r7);
      smartGraph.add(srf2);
      r8 = curr_pose.range(lmk3); % range of lmk3 wrt x5
      srf3.addRange(key_curr, r8);

      % DELAYED INITIALIZATION: 
      fullGraph.add(rangef6); 
      rangef7 = RangeFactor2D(key_curr, lmkKey(2), r7, noiseRange);
      fullGraph.add(rangef7); 
      rangef8 = RangeFactor2D(key_curr, lmkKey(3), r8, noiseRange);
      fullGraph.add(rangef8); 
      
      if goodInitFlag_lmk3==1
        fullValues.insert(lmkKey(3), lmk3);
      else
        fullValues.insert(lmkKey(3), lmk3_bad);
      end
    %====================================================================      
    case 6
      % x6-lmk3
      r9 = curr_pose.range(lmk3); % range of lmk3 wrt x6
      srf3.addRange(key_curr, r9);
      
      rangef9 = RangeFactor2D(key_curr, lmkKey(3), r9, noiseRange);
      fullGraph.add(rangef9);
    case 7
      % x6-lmk3
      r10 = curr_pose.range(lmk3); % range of lmk3 wrt x7
      srf3.addRange(key_curr, r10);
      smartGraph.add(srf3);
      
      rangef10 = RangeFactor2D(key_curr, lmkKey(3), r10, noiseRange);
      fullGraph.add(rangef10);
  end
  
  smartOpt = GaussNewtonOptimizer(smartGraph, smartValues);
  smartSolution = smartOpt.optimize;
  figure(1)
  clf
  plot2DTrajectory(smartSolution, 'b.-');
  title('Ground truth (g) VS smart estimate (b) VS full estimate (r)')
  xlabel('x')
  ylabel('y')
  zlabel('z')
  axis equal
  hold on
  
  fullOpt = GaussNewtonOptimizer(fullGraph, fullValues);
  fullSolution = fullOpt.optimize;
  figure(1)
  plot2DTrajectory(fullSolution, 'r.-');
  
  figure(1)
  plot2DTrajectory(smartValues, 'g.-');
  drawnow; 
end









