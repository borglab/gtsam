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

% Data Options
options.triangle = false;
options.nrCameras = 10;
options.showImages = false;

%% Generate data
[data,truth] = VisualISAMGenerateData(options);

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = visualSLAMGraph;

%% Add factors for all measurements
measurementNoise = gtsamSharedNoiseModel_Sigma(2,measurementNoiseSigma);
for i=1:size(data.z,1)
    for j=1:size(data.z,2)
        graph.addMeasurement(data.z{i,j}, measurementNoise, symbol('x',i), symbol('l',j), data.K);
    end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = gtsamSharedNoiseModel_Sigmas(poseNoiseSigmas);
graph.addPosePrior(symbol('x',1), truth.cameras{1}.pose, posePriorNoise);
pointPriorNoise  = gtsamSharedNoiseModel_Sigma(3,pointNoiseSigma);
graph.addPointPrior(symbol('l',1), truth.points{1}, pointPriorNoise);

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points to ground truth in this example
initialEstimate = visualSLAMValues;
for i=1:size(truth.cameras,2)
    initialEstimate.insertPose(symbol('x',i), truth.cameras{i}.pose);
end
for j=1:size(truth.points,2)
    initialEstimate.insertPoint(symbol('l',j), truth.points{j});
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate);
result.print(sprintf('\nFinal result:\n  '));

%% Plot results with covariance ellipses
marginals = graph.marginals(result);
cla
hold on;
for j=1:result.nrPoints
    P = marginals.marginalCovariance(symbol('l',j));
    point_j = result.point(symbol('l',j));
	plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end

for i=1:result.nrPoses
    P = marginals.marginalCovariance(symbol('x',i));
    pose_i = result.pose(symbol('x',i));
    plotPose3(pose_i,P,10);
end
axis([-35 35 -35 35 -15 15]);
axis equal
view(-37,40)
colormap hot
