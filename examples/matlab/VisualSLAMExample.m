%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief A simple visual SLAM example for structure from motion
% @author Duy-Nguyen Ta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - Landmarks as 8 vertices of a cube: (10,10,10) (-10,10,10) etc...
%  - Cameras are on a circle around the cube, pointing at the world origin
%  - Each camera sees all landmarks. 
%  - Visual measurements as 2D points are given, corrupted by Gaussian noise.

%% Generate simulated data
% 3D landmarks as vertices of a cube
points = {gtsamPoint3([10 10 10]'),...
    gtsamPoint3([-10 10 10]'),...
    gtsamPoint3([-10 -10 10]'),...
    gtsamPoint3([10 -10 10]'),...
    gtsamPoint3([10 10 -10]'),...
    gtsamPoint3([-10 10 -10]'),...
    gtsamPoint3([-10 -10 -10]'),...
    gtsamPoint3([10 -10 -10]')};

% Camera cameras on a circle around the cube, pointing at the world origin
nCameras = 6;
height = 0;
r = 30;
cameras = {};
K = gtsamCal3_S2(500,500,0,640/2,480/2);
for i=1:nCameras
    theta = (i-1)*2*pi/nCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), K);
end

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = visualSLAMGraph;

%% Add factors for all measurements
measurementNoise = gtsamSharedNoiseModel_Sigma(2,measurementNoiseSigma);
for i=1:nCameras
    for j=1:size(points,2)
        zij = cameras{i}.project(points{j}); % you can add noise here if desired
        graph.addMeasurement(zij, measurementNoise, symbol('x',i), symbol('l',j), K);
    end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = gtsamSharedNoiseModel_Sigmas(poseNoiseSigmas);
graph.addPosePrior(symbol('x',1), cameras{1}.pose, posePriorNoise);
pointPriorNoise  = gtsamSharedNoiseModel_Sigma(3,pointNoiseSigma);
graph.addPointPrior(symbol('l',1), points{1}, pointPriorNoise);

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy cameras and points
initialEstimate = visualSLAMValues;
for i=1:size(cameras,2)
    initialEstimate.insertPose(symbol('x',i), cameras{i}.pose);
end
for j=1:size(points,2)
    initialEstimate.insertPoint(symbol('l',j), points{j});
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate);
result.print(sprintf('\nFinal result:\n  '));

%% Plot results with covariance ellipses
marginals = graph.marginals(result);
figure(1);clf
hold on;
for j=1:size(points,2)
    P = marginals.marginalCovariance(symbol('l',j));
    point_j = result.point(symbol('l',j));
	plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end

for i=1:size(cameras,2)
    P = marginals.marginalCovariance(symbol('x',i))
    pose_i = result.pose(symbol('x',i))
    plotPose3(pose_i,P,10);
end
axis([-35 35 -35 35 -15 15]);
axis equal
view(-37,40)
