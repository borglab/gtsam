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

%% Create keys for variables
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2);

%% Create graph and factors
graph = NonlinearFactorGraph;

% Prior factor - FAIL: unregistered class
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise)); % add directly to graph

% Between Factors - FAIL: unregistered class
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise));

% BearingRange Factors - FAIL: unregistered class
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(4+4), brNoise));
graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
graph.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));

%% Create Values
values = Values;
values.insert(i1, Pose2(0.5, 0.0, 0.2));
values.insert(i2, Pose2(2.3, 0.1,-0.2));
values.insert(i3, Pose2(4.1, 0.1, 0.1));
values.insert(j1, Point2(1.8, 2.1));
values.insert(j2, Point2(4.1, 1.8));

%% Check that serialization works properly
serialized_values = serializeValues(values); % returns a string
deserializedValues = deserializeValues(serialized_values); % returns a new values
CHECK('values.equals(deserializedValues)',values.equals(deserializedValues,1e-9));

CHECK('serializeValuesToFile(values, values.dat)', serializeValuesToFile(values, 'values.dat')); 
deserializedValuesFile = deserializeValuesFromFile('values.dat'); % returns a new values
CHECK('values.equals(deserializedValuesFile)',values.equals(deserializedValuesFile,1e-9));

% % FAIL: unregistered class - derived class not registered or exported
% serialized_graph = serializeGraph(graph); % returns a string
% deserializedGraph = deserializeGraph(serialized_graph); % returns a new graph
% CHECK('graph.equals(deserializedGraph)',graph.equals(deserializedGraph,1e-9));
%
% CHECK('serializeGraphToFile(graph, graph.dat)', serializeGraphToFile(graph, 'graph.dat')); 
% deserializedGraphFile = deserializeGraphFromFile('graph.dat'); % returns a new graph
% CHECK('graph.equals(deserializedGraphFile)',graph.equals(deserializedGraphFile,1e-9));