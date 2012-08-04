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

%% Find data file
datafile = gtsam.findExampleDataFile('w100-odom.graph');

%% Initialize graph, initial estimate, and odometry noise
import gtsam.*
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 5*pi/180]);
maxID = 0;
addNoise = false;
smart = true;
[graph,initial] = gtsam.load2D(datafile, model, maxID, addNoise, smart);
initial.print(sprintf('Initial estimate:\n'));

%% Add a Gaussian prior on pose x_1
import gtsam.*
priorMean = Pose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise)); % add directly to graph

%% Plot Initial Estimate
import gtsam.*
cla
gtsam.plot2DTrajectory(initial, 'g-*'); axis equal

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
import gtsam.*
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely;
hold on; gtsam.plot2DTrajectory(result, 'b-*');
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
import gtsam.*
marginals = Marginals(graph, result);
P={};
for i=1:result.size()-1
    pose_i = result.at(i);
    P{i}=marginals.marginalCovariance(i);
    gtsam.plotPose2(pose_i,'b',P{i})
end
view(2)
axis([-15 10 -15 10]); axis equal;
fprintf(1,'%.5f %.5f %.5f\n',P{99})