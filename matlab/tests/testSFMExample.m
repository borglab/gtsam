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

options.triangle = false;
options.nrCameras = 10;
options.showImages = false;

[data,truth] = VisualISAMGenerateData(options);

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

graph = visualSLAMGraph;

%% Add factors for all measurements
measurementNoise = gtsamnoiseModelIsotropic_Sigma(2,measurementNoiseSigma);
for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.addMeasurement(data.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), data.K);
    end
end

posePriorNoise  = gtsamnoiseModelDiagonal_Sigmas(poseNoiseSigmas);
graph.addPosePrior(symbol('x',1), truth.cameras{1}.pose, posePriorNoise);
pointPriorNoise  = gtsamnoiseModelIsotropic_Sigma(3,pointNoiseSigma);
graph.addPointPrior(symbol('p',1), truth.points{1}, pointPriorNoise);

%% Initial estimate
initialEstimate = visualSLAMValues;
for i=1:size(truth.cameras,2)
    pose_i = truth.cameras{i}.pose;
    initialEstimate.insertPose(symbol('x',i), pose_i);
end
for j=1:size(truth.points,2)
    point_j = truth.points{j};
    initialEstimate.insertPoint(symbol('p',j), point_j);
end

%% Optimization
parameters = gtsamLevenbergMarquardtParams(1e-5, 1e-5, 0, 0);
optimizer = graph.optimizer(initialEstimate, parameters);
for i=1:5
    optimizer.iterate();
end
result = optimizer.values();

%% Marginalization
marginals = graph.marginals(result);
marginals.marginalCovariance(symbol('p',1));
marginals.marginalCovariance(symbol('x',1));

%% Check optimized results, should be equal to ground truth
for i=1:size(truth.cameras,2)
    pose_i = result.pose(symbol('x',i));
    CHECK('pose_i.equals(truth.cameras{i}.pose,1e-5)',pose_i.equals(truth.cameras{i}.pose,1e-5))
end

for j=1:size(truth.points,2)
    point_j = result.point(symbol('p',j));
    CHECK('point_j.equals(truth.points{j},1e-5)',point_j.equals(truth.points{j},1e-5))
end
