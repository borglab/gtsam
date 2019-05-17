% ----------------------------------------------------------------------------
%
%  GTSAM Copyright 2010, Georgia Tech Research Corporation,
%  Atlanta, Georgia 30332-0415
%  All Rights Reserved
%  Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
%  See LICENSE for the license information
%
%  --------------------------------------------------------------------------

%  @file ConcurrentFilteringAndSmoothingExample.m
%  @brief Demonstration of the concurrent filtering and smoothing architecture using
%  a planar robot example and multiple odometry-like sensors
%  @author Stephen Williams

%  A simple 2D pose slam example with multiple odometry-like measurements
%   - The robot initially faces along the X axis (horizontal, to the right in 2D)
%   - The robot moves forward at 2m/s
%   - We have measurements between each pose from multiple odometry sensors

clear all;
clear all;
import gtsam.*;

% Define the smoother lag (in seconds)
lag = 2.0;

% Create a Concurrent Filter and Smoother
concurrentFilter = ConcurrentBatchFilter;
concurrentSmoother = ConcurrentBatchSmoother;

%% Create containers to store the factors and linearization points that
% will be sent to the smoothers
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Create a prior on the first pose, placing it at the origin
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3 ; 0.3 ; 0.1]);
priorKey = uint64(0);
newFactors.add(PriorFactorPose2(priorKey, priorMean, priorNoise));
newValues.insert(priorKey, priorMean); % Initialize the first pose at the mean of the prior

%% Insert the prior factor into the filter
concurrentFilter.update(newFactors, newValues);

%% Now, loop through several time steps, creating factors from different "sensors"
% and adding them to the fixed-lag smoothers
deltaT = 0.25;
for time = deltaT : deltaT : 10.0
  
  %% Initialize factor and values for this loop iteration
  newFactors = NonlinearFactorGraph;
  newValues = Values;
  
  %% Define the keys related to this timestamp
  previousKey = uint64(1000 * (time-deltaT));
  currentKey = uint64(1000 * (time));
  
  %% Add a guess for this pose to the new values
  % Since the robot moves forward at 2 m/s, then the position is simply: time[s]*2.0[m/s]
  % {This is not a particularly good way to guess, but this is just an example}
  currentPose = Pose2(time * 2.0, 0.0, 0.0);
  newValues.insert(currentKey, currentPose);
  
  %% Add odometry factors from two different sources with different error stats
  odometryMeasurement1 = Pose2(0.61, -0.08, 0.02);
  odometryNoise1 = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.05]);
  newFactors.add(BetweenFactorPose2(previousKey, currentKey, odometryMeasurement1, odometryNoise1));
  
  odometryMeasurement2 = Pose2(0.47, 0.03, 0.01);
  odometryNoise2 = noiseModel.Isotropic.Sigma(3, 0.05);
  newFactors.add(BetweenFactorPose2(previousKey, currentKey, odometryMeasurement2, odometryNoise2));
  
  %% Unlike the fixed-lag versions, the concurrent filter implementation
  % requires the user to supply the specify which keys to move to the smoother
  oldKeys = KeyList;
  if time >= lag+deltaT
    oldKeys.push_back(uint64(1000 * (time-lag-deltaT)));
  end
  
  %% Update the various inference engines
  concurrentFilter.update(newFactors, newValues, oldKeys);
  
  %% Add a loop closure to the smoother at a particular time
  if time == 8.0
    % Now lets create a "loop closure" factor between some poses
    loopKey1 = uint64(1000 * (0.0));
    loopKey2 = uint64(1000 * (5.0));
    loopMeasurement = Pose2(9.5, 1.00, 0.00);
    loopNoise = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.25]);
    loopFactor = BetweenFactorPose2(loopKey1, loopKey2, loopMeasurement, loopNoise);
    % The state at 5.0s should have been transferred to the concurrent smoother at this point. Add the loop closure.
    smootherFactors = NonlinearFactorGraph;
    smootherFactors.add(loopFactor);
    concurrentSmoother.update(smootherFactors, Values());
  end
  
  %% Manually synchronize the Concurrent Filter and Smoother every 1.0 s
  if mod(time, 1.0) < 0.01
    % Synchronize the Filter and Smoother
    concurrentSmoother.update();
    synchronize(concurrentFilter, concurrentSmoother);
  end
  
  %% Print the filter optimized poses
  fprintf(1, 'Timestamp = %5.3f\n', time);
  filterResult = concurrentFilter.calculateEstimate;
  filterResult.atPose2(currentKey).print('Concurrent Estimate: ');
  
  %% Plot Covariance Ellipses
  cla;
  hold on
  filterMarginals = Marginals(concurrentFilter.getFactors, filterResult);
  plot2DTrajectory(filterResult, 'k*-', filterMarginals);
  
  smootherGraph = concurrentSmoother.getFactors;
  if smootherGraph.size > 0
    smootherResult = concurrentSmoother.calculateEstimate;
    smootherMarginals = Marginals(smootherGraph, smootherResult);
    plot2DTrajectory(smootherResult, 'r*-', smootherMarginals);
  end
  
  axis equal
  axis tight
  view(2)
  pause(0.01)
end



