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
import gtsam.*
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 1*pi/180]);
maxID=0;
addNoise=false;
smart=true;
[graph,initial]=load2D('Data/w10000-odom.graph',model,maxID,addNoise,smart);

%% Add a Gaussian prior on pose x_1
priorMean = Pose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.addPosePrior(0, priorMean, priorNoise); % add directly to graph

%% Plot Initial Estimate
figure(1);clf
P=initial.poses;
plot(P(:,1),P(:,2),'r-'); axis equal

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
tic
result = graph.optimize(initial,1);
toc
P=result.poses;
hold on; plot(P(:,1),P(:,2),'b-')
