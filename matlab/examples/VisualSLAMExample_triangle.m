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

%% Create a triangle target, just 3 points on a plane
r = 10;
points = {};
for j=1:3
    theta = (j-1)*2*pi/3;
    points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
end

%% Create camera cameras on a circle around the triangle
nCameras = 6;
height = 10;
r = 30;
cameras = {};
K = gtsamCal3_S2(500,500,0,640/2,480/2);
for i=1:nCameras
    theta = (i-1)*2*pi/nCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), K)
end

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = visualSLAMGraph;

%% Add factors for all measurements
measurementNoiseSigma=1; % in pixels
measurementNoise = gtsamSharedNoiseModel_Sigma(2,measurementNoiseSigma);
for i=1:nCameras
    for j=1:3
        zij = cameras{i}.project(points{j}); % you can add noise here if desired
        graph.addMeasurement(zij, measurementNoise, symbol('x',i), symbol('l',j), K);
    end
end

%% Add Gaussian priors for 3 points to constrain the system
pointPriorNoise  = gtsamSharedNoiseModel_Sigma(3,0.1);
for j=1:3
    graph.addPointPrior(symbol('l',j), points{j}, pointPriorNoise);
end

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
axis([-20 20 -20 20 -1 15]);
axis equal
view(-37,40)
