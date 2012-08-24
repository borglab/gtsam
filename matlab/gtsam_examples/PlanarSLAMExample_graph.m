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
datafile = '/Users/dellaert/borg/gtsam/examples/Data/example.graph';

%% Initialize graph, initial estimate, and odometry noise
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 2*pi/180]);
[graph,initial] = load2D(datafile, model);

%% Add a Gaussian prior on pose x_1
priorMean = initial.at(0);
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise)); % add directly to graph

%% Plot Initial Estimate
cla
plot2DTrajectory(initial, 'g-*'); axis equal

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initial);
tic
result = optimizer.optimizeSafely;
toc
hold on; plot2DTrajectory(result, 'b-*');

%% Plot Covariance Ellipses
marginals = Marginals(graph, result);
P={};
for i=0:94
    pose_i = result.at(i);
    Pi=marginals.marginalCovariance(i);
    plotPose2(pose_i,'b',Pi)
end
view(2)
axis tight; axis equal;
