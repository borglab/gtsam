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

%% Find data file
datafile = findExampleDataFile('w100.graph');

%% Initialize graph, initial estimate, and odometry noise
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 5*pi/180]);
[graph,initial] = load2D(datafile, model);

%% Add a Gaussian prior on pose x_1
priorMean = Pose2(0, 0, 0); % prior mean is at origin
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
tic
marginals = Marginals(graph, result);
toc
P={};
for i=1:result.size()-1
    pose_i = result.atPose2(i);
    P{i}=marginals.marginalCovariance(i);
    plotPose2(pose_i,'b',P{i})
end
view(2)
axis tight; axis equal;
% fprintf(1,'%.5f %.5f %.5f\n',P{99})