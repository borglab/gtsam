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

%% Create keys for variables
i1 = gtsam.symbol('x',1); i2 = gtsam.symbol('x',2); i3 = gtsam.symbol('x',3);
j1 = gtsam.symbol('l',1); j2 = gtsam.symbol('l',2);

%% Create values and verify string serialization
pose1=gtsam.Pose2(0.5, 0.0, 0.2);
pose2=gtsam.Pose2(2.3, 0.1,-0.2);
pose3=gtsam.Pose2(4.1, 0.1, 0.1);
landmark1=gtsam.Point2(1.8, 2.1);
landmark2=gtsam.Point2(4.1, 1.8);

serialized_pose1 = pose1.string_serialize();
pose1ds = gtsam.Pose2.string_deserialize(serialized_pose1);
gtsam.CHECK('pose1ds.equals(pose1, 1e-9)', pose1ds.equals(pose1, 1e-9));

%% Create and serialize Values
values = gtsam.Values;
values.insert(i1, pose1);
values.insert(i2, pose2);
values.insert(i3, pose3);
values.insert(j1, landmark1);
values.insert(j2, landmark2);

serialized_values = values.string_serialize();
valuesds = gtsam.Values.string_deserialize(serialized_values);
gtsam.CHECK('valuesds.equals(values, 1e-9)', valuesds.equals(values, 1e-9));

%% Create graph and factors and serialize
graph = gtsam.NonlinearFactorGraph;

% Prior factor
priorMean = gtsam.Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(gtsam.PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph

% Between Factors
odometry = gtsam.Pose2(2.0, 0.0, 0.0);
odometryNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(gtsam.BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(gtsam.BetweenFactorPose2(i2, i3, odometry, odometryNoise));
 
% Range Factors
rNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2]);
graph.add(gtsam.RangeFactor2D(i1, j1, sqrt(4+4), rNoise));
graph.add(gtsam.RangeFactor2D(i2, j1, 2, rNoise));
graph.add(gtsam.RangeFactor2D(i3, j2, 2, rNoise));

% Bearing Factors
degrees = pi/180;
bNoise = gtsam.noiseModel.Diagonal.Sigmas([0.1]);
graph.add(gtsam.BearingFactor2D(i1, j1, gtsam.Rot2(45*degrees), bNoise));
graph.add(gtsam.BearingFactor2D(i2, j1, gtsam.Rot2(90*degrees), bNoise));
graph.add(gtsam.BearingFactor2D(i3, j2, gtsam.Rot2(90*degrees), bNoise));

% BearingRange Factors
brNoise = gtsam.noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(gtsam.BearingRangeFactor2D( ...
    i1, j1, gtsam.Rot2(45*degrees), sqrt(4+4), brNoise));
graph.add(gtsam.BearingRangeFactor2D( ...
    i2, j1, gtsam.Rot2(90*degrees), 2, brNoise));
graph.add(gtsam.BearingRangeFactor2D( ...
    i3, j2, gtsam.Rot2(90*degrees), 2, brNoise));

serialized_graph = graph.string_serialize();
graphds = gtsam.NonlinearFactorGraph.string_deserialize(serialized_graph);
gtsam.CHECK('graphds.equals(graph, 1e-9)', graphds.equals(graph, 1e-9));
