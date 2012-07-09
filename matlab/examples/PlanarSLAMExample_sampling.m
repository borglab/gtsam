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

%% Create the same factor graph as in PlanarSLAMExample
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
graph = planarSLAMGraph;
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsamnoiseModelDiagonal.Sigmas([0.3; 0.3; 0.1]);
graph.addPosePrior(i1, priorMean, priorNoise); % add directly to graph
odometry = gtsamPose2(2.0, 0.0, 0.0);
odometryNoise = gtsamnoiseModelDiagonal.Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(i1, i2, odometry, odometryNoise);
graph.addRelativePose(i2, i3, odometry, odometryNoise);

%% Except, for measurements we offer a choice
j1 = symbol('l',1); j2 = symbol('l',2);
degrees = pi/180;
noiseModel = gtsamnoiseModelDiagonal.Sigmas([0.1; 0.2]);
if 1
    graph.addBearingRange(i1, j1, gtsamRot2(45*degrees), sqrt(4+4), noiseModel);
    graph.addBearingRange(i2, j1, gtsamRot2(90*degrees), 2, noiseModel);
else
    bearingModel = gtsamnoiseModelDiagonal.Sigmas(0.1);    
    graph.addBearing(i1, j1, gtsamRot2(45*degrees), bearingModel);
    graph.addBearing(i2, j1, gtsamRot2(90*degrees), bearingModel);
end
graph.addBearingRange(i3, j2, gtsamRot2(90*degrees), 2, noiseModel);    

%% Initialize MCMC sampler with ground truth
sample = planarSLAMValues;
sample.insertPose(i1, gtsamPose2(0,0,0));
sample.insertPose(i2, gtsamPose2(2,0,0));
sample.insertPose(i3, gtsamPose2(4,0,0));
sample.insertPoint(j1, gtsamPoint2(2,2));
sample.insertPoint(j2, gtsamPoint2(4,2));

%% Calculate and plot Covariance Ellipses
figure(1);clf;hold on
marginals = graph.marginals(sample);
for i=1:3
    key = symbol('x',i);
    pose{i} = sample.pose(key);
    P{i}=marginals.marginalCovariance(key);
    if i>1
        plot([pose{i-1}.x;pose{i}.x],[pose{i-1}.y;pose{i}.y],'r-');
    end
end
for i=1:3
    plotPose2(pose{i},'g',P{i})
end
for j=1:2
    key = symbol('l',j);
    point{j} = sample.point(key);
    Q{j}=marginals.marginalCovariance(key);
    S{j}=chol(Q{j}); % for sampling
    plotPoint2(point{j},'b',Q{j})
end
plot([pose{1}.x;point{1}.x],[pose{1}.y;point{1}.y],'c-');
plot([pose{2}.x;point{1}.x],[pose{2}.y;point{1}.y],'c-');
plot([pose{3}.x;point{2}.x],[pose{3}.y;point{2}.y],'c-');
axis equal

%% Do Sampling on point 2
N=1000;
for s=1:N
    delta = S{2}*randn(2,1);
    proposedPoint = gtsamPoint2(point{2}.x+delta(1),point{2}.y+delta(2));
    plotPoint2(proposedPoint,'k.')
end