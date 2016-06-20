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

clear

import gtsam.*

%% Find data file
datafile = findExampleDataFile('example.graph');

%% Initialize graph, initial estimate, and odometry noise
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 2*pi/180]);
[graph,initial] = load2D(datafile, model);

%% Add a Gaussian prior on a pose in the middle
priorMean = initial.atPose2(40);
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 2*pi/180]);
graph.add(PriorFactorPose2(40, priorMean, priorNoise)); % add directly to graph

%% Plot Initial Estimate
cla
plot2DTrajectory(initial, 'r-'); axis equal

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initial);
tic
result = optimizer.optimizeSafely;
toc

%% Plot Covariance Ellipses
cla;hold on
marginals = Marginals(graph, result);
plot2DTrajectory(result, 'g', marginals);
plot2DPoints(result, 'b', marginals);
axis tight
axis equal
view(2)

