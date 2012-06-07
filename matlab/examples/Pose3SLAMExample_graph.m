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

N = 2500;
% filename = '../../examples/Data/sphere_smallnoise.graph';
% filename = '../../examples/Data/sphere2500_groundtruth.txt';
filename = '../../examples/Data/sphere2500.txt';

%% Initialize graph, initial estimate, and odometry noise
model = gtsamSharedNoiseModel_Sigmas([0.05; 0.05; 0.05; 5*pi/180; 5*pi/180; 5*pi/180]);
[graph,initial]=load3D(filename,model,true,N);

%% Plot Initial Estimate
figure(1);clf
first = initial.pose(0);
plot3(first.x(),first.y(),first.z(),'r*'); hold on
plot3DTrajectory(initial,'g-',false);

%% Read again, now with all constraints, and optimize
graph = load3D(filename,model,false,N);
graph.addHardConstraint(0, first);
result = graph.optimize(initial);
plot3DTrajectory(result,'r-',false); axis equal;
