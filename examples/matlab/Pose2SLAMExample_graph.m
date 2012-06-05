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

%% Initialize graph, initial estimate, and odometry noise
model = gtsamSharedNoiseModel_Sigmas([0.05; 0.05; 5*pi/180]);
[graph,initial]=load2D('../Data/w100-odom.graph',model);
initial.print(sprintf('Initial estimate:\n'));

%% Add a Gaussian prior on pose x_1
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = gtsamSharedNoiseModel_Sigmas([0.01; 0.01; 0.01]);
graph.addPrior(0, priorMean, priorNoise); % add directly to graph

%% Plot Initial Estimate
figure(1);clf
plot(initial.xs(),initial.ys(),'g-*'); axis equal

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initial);
hold on; plot(result.xs(),result.ys(),'b-*')
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
marginals = graph.marginals(result);
for i=1:result.size()-1
    pose_i = result.pose(i);
    P{i}=marginals.marginalCovariance(i);
    plotPose2(pose_i,'b',P{i})
end
fprintf(1,'%.5f %.5f %.5f\n',P{99})