%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Checks for serialization using basic string interface
% @author Alex Cunningham
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Create keys for variables
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2);

%% Create values and verify string serialization
pose1=Pose2(0.5, 0.0, 0.2);
pose2=Pose2(2.3, 0.1,-0.2);
pose3=Pose2(4.1, 0.1, 0.1);
landmark1=Point2(1.8, 2.1);
landmark2=Point2(4.1, 1.8);

serialized_pose1 = pose1.string_serialize();
pose1ds = Pose2.string_deserialize(serialized_pose1);
CHECK('pose1ds.equals(pose1, 1e-9)', pose1ds.equals(pose1, 1e-9));

%% Create and serialize Values
values = Values;
values.insert(i1, pose1);
values.insert(i2, pose2);
values.insert(i3, pose3);
values.insert(j1, landmark1);
values.insert(j2, landmark2);

serialized_values = values.string_serialize();
valuesds = Values.string_deserialize(serialized_values);
CHECK('valuesds.equals(values, 1e-9)', valuesds.equals(values, 1e-9));

%% Create graph and factors and serialize
graph = NonlinearFactorGraph;

% Prior factor
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph

% Between Factors
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise));
 
% Range Factors
rNoise = noiseModel.Diagonal.Sigmas([0.2]);
graph.add(RangeFactor2D(i1, j1, sqrt(4+4), rNoise));
graph.add(RangeFactor2D(i2, j1, 2, rNoise));
graph.add(RangeFactor2D(i3, j2, 2, rNoise));

% Bearing Factors
degrees = pi/180;
bNoise = noiseModel.Diagonal.Sigmas([0.1]);
graph.add(BearingFactor2D(i1, j1, Rot2(45*degrees), bNoise));
graph.add(BearingFactor2D(i2, j1, Rot2(90*degrees), bNoise));
graph.add(BearingFactor2D(i3, j2, Rot2(90*degrees), bNoise));

% BearingRange Factors
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(4+4), brNoise));
graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
graph.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));

serialized_graph = graph.string_serialize();
graphds = NonlinearFactorGraph.string_deserialize(serialized_graph);
CHECK('graphds.equals(graph, 1e-9)', graphds.equals(graph, 1e-9));