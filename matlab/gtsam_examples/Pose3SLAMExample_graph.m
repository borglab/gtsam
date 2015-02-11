%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% PLEASE NOTE THAT PLOTTING TAKES A VERY LONG TIME HERE

%% Find data file
N = 2500;
% dataset = 'sphere_smallnoise.graph';
% dataset = 'sphere2500_groundtruth.txt';
dataset = 'sphere2500.txt';

datafile = findExampleDataFile(dataset);

%% Initialize graph, initial estimate, and odometry noise
model = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
[graph,initial]=load3D(datafile,model,true,N);

%% Plot Initial Estimate
cla
first = initial.atPose3(0);
plot3(first.x(),first.y(),first.z(),'r*'); hold on
plot3DTrajectory(initial,'g-',false);
drawnow;

%% Read again, now with all constraints, and optimize
graph = load3D(datafile, model, false, N);
graph.add(NonlinearEqualityPose3(0, first));
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely();
plot3DTrajectory(result, 'r-', false); axis equal;

view(3); axis equal;