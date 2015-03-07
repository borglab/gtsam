%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Simple robotics example using the pre-built planar SLAM domain
% @author Alex Cunningham
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Create the same factor graph as in PlanarSLAMExample
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
graph = NonlinearFactorGraph;
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise));

%% Except, for measurements we offer a choice
j1 = symbol('l',1); j2 = symbol('l',2);
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
if 1
    graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(4+4), brNoise));
    graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
else
    bearingModel = noiseModel.Diagonal.Sigmas(0.1);    
    graph.add(BearingFactor2D(i1, j1, Rot2(45*degrees), bearingModel));
    graph.add(BearingFactor2D(i2, j1, Rot2(90*degrees), bearingModel));
end
graph.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));

%% Initialize MCMC sampler with ground truth
sample = Values;
sample.insert(i1, Pose2(0,0,0));
sample.insert(i2, Pose2(2,0,0));
sample.insert(i3, Pose2(4,0,0));
sample.insert(j1, Point2(2,2));
sample.insert(j2, Point2(4,2));

%% Calculate and plot Covariance Ellipses
cla;hold on
marginals = Marginals(graph, sample);

plot2DTrajectory(sample, [], marginals);
plot2DPoints(sample, [], marginals);

for j=1:2
    key = symbol('l',j);
    point{j} = sample.at(key);
    Q{j}=marginals.marginalCovariance(key);
    S{j}=chol(Q{j}); % for sampling
end

plot([sample.at(i1).x; sample.at(j1).x],[sample.at(i1).y; sample.at(j1).y], 'c-');
plot([sample.at(i2).x; sample.at(j1).x],[sample.at(i2).y; sample.at(j1).y], 'c-');
plot([sample.at(i3).x; sample.at(j2).x],[sample.at(i3).y; sample.at(j2).y], 'c-');
view(2); axis auto; axis equal

%% Do Sampling on point 2
N=1000;
for s=1:N
    delta = S{2}*randn(2,1);
    proposedPoint = Point2(point{2}.x+delta(1),point{2}.y+delta(2));
    plotPoint2(proposedPoint,'k.')
end