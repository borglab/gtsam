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

import gtsam.*

options.triangle = false;
options.nrCameras = 10;
options.showImages = false;

[data,truth] = support.VisualISAMGenerateData(options);

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

graph = NonlinearFactorGraph;

%% Add factors for all measurements
import gtsam.*
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.add(GenericProjectionFactorCal3_S2(data.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), data.K));
    end
end

posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph.add(PriorFactorPose3(symbol('x',1), truth.cameras{1}.pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph.add(PriorFactorPose3(symbol('p',1), truth.points{1}, pointPriorNoise));

%% Initial estimate
initialEstimate = Values;
for i=1:size(truth.cameras,2)
    pose_i = truth.cameras{i}.pose;
    initialEstimate.insert(symbol('x',i), pose_i);
end
for j=1:size(truth.points,2)
    point_j = truth.points{j};
    initialEstimate.insert(symbol('p',j), point_j);
end

%% Optimization
import gtsam.*
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
for i=1:5
    optimizer.iterate();
end
result = optimizer.values();

%% Marginalization
marginals = Marginals(graph, result);
marginals.marginalCovariance(symbol('p',1));
marginals.marginalCovariance(symbol('x',1));

%% Check optimized results, should be equal to ground truth
keys = truth.keys;
for i=0:keys.size-1
    truth_i = truth.at(keys.at(i));
    result_i = result.at(keys.at(i));
    CHECK('result_i.equals(truth_i,1e-5)',result_i.equals(truth_i,1e-5))
end
