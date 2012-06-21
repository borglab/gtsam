%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief A structure from motion example
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
for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.addMeasurement(data.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), data.K);
    end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = gtsamSharedNoiseModel_Sigmas(poseNoiseSigmas);
graph.addPosePrior(symbol('x',1), truth.cameras{1}.pose, posePriorNoise);
pointPriorNoise  = gtsamSharedNoiseModel_Sigma(3,pointNoiseSigma);
graph.addPointPrior(symbol('p',1), truth.points{1}, pointPriorNoise);

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points close to ground truth in this example
initialEstimate = visualSLAMValues;
for i=1:size(truth.cameras,2)
    pose_i = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    initialEstimate.insertPose(symbol('x',i), pose_i);
end
for j=1:size(truth.points,2)
    point_j = truth.points{j}.retract(0.1*randn(3,1));
    initialEstimate.insertPoint(symbol('p',j), point_j);
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% One-shot Optimize using Levenberg-Marquardt optimization with an ordering from colamd
%result = graph.optimize(initialEstimate,1);
%result.print(sprintf('\nFinal result:\n  '));

%% Fine grain optimization, allowing user to iterate step by step

parameters = gtsamLevenbergMarquardtParams(1e-5, 1e-5, 0, 2);
optimizer = graph.optimizer(initialEstimate, parameters);
for i=1:5
    optimizer.iterate();
end
result = optimizer.values();
result.print(sprintf('\nFinal result:\n  '));

%% Plot results with covariance ellipses
marginals = graph.marginals(result);
cla
hold on;
for j=1:result.nrPoints
    P = marginals.marginalCovariance(symbol('p',j));
    point_j = result.point(symbol('p',j));
	plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end

for i=1:result.nrPoses
    P = marginals.marginalCovariance(symbol('x',i));
    pose_i = result.pose(symbol('x',i));
    plotPose3(pose_i,P,10);
end
axis([-40 40 -40 40 -10 20]);axis equal
view(3)
colormap('hot')
