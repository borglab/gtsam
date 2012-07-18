%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief An SFM example (adapted from SFMExample.m) optimizing calibration
% @author Yong-Dian Jian
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
cameraNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1 ...
                     0.001*ones(1,5)]';

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = sparseBA.Graph;

 
%% Add factors for all measurements
import gtsam.*
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.addSimpleCameraMeasurement(data.Z{i}{k}, measurementNoise, symbol('c',i), symbol('p',j));
    end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
import gtsam.*
cameraPriorNoise  = noiseModel.Diagonal.Sigmas(cameraNoiseSigmas);
firstCamera = SimpleCamera(truth.cameras{1}.pose, truth.K);
graph.addSimpleCameraPrior(symbol('c',1), firstCamera, cameraPriorNoise);

pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph.addPointPrior(symbol('p',1), truth.points{1}, pointPriorNoise);

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));


%% Initialize cameras and points close to ground truth in this example
import gtsam.*
initialEstimate = sparseBA.Values;
for i=1:size(truth.cameras,2)
    pose_i = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    camera_i = SimpleCamera(pose_i, truth.K);
    initialEstimate.insertSimpleCamera(symbol('c',i), camera_i);
end
for j=1:size(truth.points,2)
    point_j = truth.points{j}.retract(0.1*randn(3,1));
    initialEstimate.insertPoint(symbol('p',j), point_j);
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Fine grain optimization, allowing user to iterate step by step

import gtsam.*
parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = graph.optimizer(initialEstimate, parameters);

for i=1:5
    optimizer.iterate();
end

result = optimizer.values();
result.print(sprintf('\nFinal result:\n  '));


