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

% Camera poses on a circle around the cube, pointing at the world origin
nCameras = 8;
r = 30;
poses = {};
for i=1:nCameras
    theta = i*2*pi/nCameras;
    posei = gtsamPose3(...
                gtsamRot3([-sin(theta) 0 -cos(theta); 
                            cos(theta) 0 -sin(theta); 
                            0 -1 0]),...
                gtsamPoint3([r*cos(theta), r*sin(theta), 0]'));
    poses = [poses {posei}];
end

% 2D visual measurements, simulated with Gaussian noise
z = {};
measurementNoiseSigmas = [0.5,0.5]';
measurementNoiseSampler = gtsamSharedDiagonal(measurementNoiseSigmas);
K = gtsamCal3_S2(50,50,0,50,50);
for i=1:size(poses,2)
    zi = {};
    camera = gtsamSimpleCamera(K,poses{i});
    for j=1:size(points,2)
        zi = [zi {camera.project(points{j}).compose(gtsamPoint2(measurementNoiseSampler.sample()))}];
    end
    z = [z; zi];
end

pointNoiseSigmas = [0.1,0.1,0.1]';
pointNoiseSampler = gtsamSharedDiagonal(pointNoiseSigmas);

poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
poseNoiseSampler = gtsamSharedDiagonal(poseNoiseSigmas);

hold off;

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = visualSLAMGraph;

%% Add factors for all measurements
measurementNoise = gtsamSharedNoiseModel_Sigmas(measurementNoiseSigmas);
for i=1:size(z,1)
    for j=1:size(z,2)
        graph.addMeasurement(z{i,j}, measurementNoise, symbol('x',i), symbol('l',j), K);
    end
end

%% Add Gaussian priors for a pose and a landmark to constraint the system
posePriorNoise  = gtsamSharedNoiseModel_Sigmas(poseNoiseSigmas);
graph.addPosePrior(symbol('x',1), poses{1}, posePriorNoise);
pointPriorNoise  = gtsamSharedNoiseModel_Sigmas(pointNoiseSigmas);
graph.addPointPrior(symbol('l',1), points{1}, pointPriorNoise);

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy poses and points
initialEstimate = visualSLAMValues;
for i=1:size(poses,2)
    initialEstimate.insertPose(symbol('x',i), poses{i}.compose(gtsamPose3_Expmap(poseNoiseSampler.sample())));
end
for j=1:size(points,2)
    initialEstimate.insertPoint(symbol('l',j), points{j}.compose(gtsamPoint3(pointNoiseSampler.sample())));
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate);
result.print(sprintf('\nFinal result:\n  '));

%% Query the marginals
marginals = graph.marginals(result);

%% Plot results with covariance ellipses
hold on;
for j=1:size(points,2)
    P = marginals.marginalCovariance(symbol('l',j));
    point_j = result.point(symbol('l',j));
	plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end

for i=1:size(poses,2)
    P = marginals.marginalCovariance(symbol('x',i));
    posei = result.pose(symbol('x',i))
    plotCamera(posei,10);
    posei_t = posei.translation()
    covarianceEllipse3D([posei_t.x;posei_t.y;posei_t.z],P(4:6,4:6));
end

