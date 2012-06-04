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
model = gtsamSharedNoiseModel_Sigmas([0.05; 0.05; 0.05; 5*pi/180; 5*pi/180; 5*pi/180]);
[graph,initial]=load3D('../Data/sphere_smallnoise.graph',model,100);

%% Fix first pose
first = initial.pose(0);
graph.addHardConstraint(0, first); % add directly to graph

%% Plot Initial Estimate
figure(1);clf
plot3(initial.xs(),initial.ys(),initial.zs(),'g-'); hold on
plot3(first.x(),first.y(),first.z(),'r*'); 

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initial);
hold on; plot3(result.xs(),result.ys(),result.zs(),'b-');axis equal; 