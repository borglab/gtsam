%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief FAST-Sync examples for every supported matrix Lie group.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

key0 = 10;
key1 = 42;

%% Rot2
rot2_0 = Rot2(0.2);
rot2_1 = Rot2(0.7);
rot2_result = synchronizePair(rot2_0, rot2_1, @BetweenFactorRot2, ...
    @PriorFactorRot2, 1, @fastSyncRot2, key0, key1);
assert(rot2_0.equals(rot2_result.atRot2(key0), 1e-8));
assert(rot2_1.equals(rot2_result.atRot2(key1), 1e-8));

%% Rot3
rot3_0 = Rot3.Expmap([0.1; -0.2; 0.05]);
rot3_1 = Rot3.Expmap([-0.3; 0.1; 0.2]);
rot3_result = synchronizePair(rot3_0, rot3_1, @BetweenFactorRot3, ...
    @PriorFactorRot3, 3, @fastSyncRot3, key0, key1);
assert(rot3_0.equals(rot3_result.atRot3(key0), 1e-8));
assert(rot3_1.equals(rot3_result.atRot3(key1), 1e-8));

%% Pose2
pose2_0 = Pose2(1.0, -2.0, 0.2);
pose2_1 = Pose2(2.0, 0.5, 0.7);
pose2_result = synchronizePair(pose2_0, pose2_1, @BetweenFactorPose2, ...
    @PriorFactorPose2, 3, @fastSyncPose2, key0, key1);
assert(pose2_0.equals(pose2_result.atPose2(key0), 1e-8));
assert(pose2_1.equals(pose2_result.atPose2(key1), 1e-8));

%% Pose3
pose3_0 = Pose3(rot3_0, [1.0; -2.0; 0.5]);
pose3_1 = Pose3(rot3_1, [2.0; 0.5; -1.0]);
pose3_result = synchronizePair(pose3_0, pose3_1, @BetweenFactorPose3, ...
    @PriorFactorPose3, 6, @fastSyncPose3, key0, key1);
assert(pose3_0.equals(pose3_result.atPose3(key0), 1e-8));
assert(pose3_1.equals(pose3_result.atPose3(key1), 1e-8));

%% Similarity2
similarity2_0 = Similarity2(rot2_0, [1.0; -2.0], 1.1);
similarity2_1 = Similarity2(rot2_1, [2.0; 0.5], 0.9);
similarity2_result = synchronizePair(similarity2_0, similarity2_1, ...
    @BetweenFactorSimilarity2, @PriorFactorSimilarity2, 4, ...
    @fastSyncSimilarity2, key0, key1);
assert(similarity2_0.equals(similarity2_result.atSimilarity2(key0), 1e-7));
assert(similarity2_1.equals(similarity2_result.atSimilarity2(key1), 1e-7));

%% Similarity3
similarity3_0 = Similarity3(rot3_0, [1.0; -2.0; 0.5], 1.1);
similarity3_1 = Similarity3(rot3_1, [2.0; 0.5; -1.0], 0.9);
similarity3_result = synchronizePair(similarity3_0, similarity3_1, ...
    @BetweenFactorSimilarity3, @PriorFactorSimilarity3, 7, ...
    @fastSyncSimilarity3, key0, key1);
assert(similarity3_0.equals(similarity3_result.atSimilarity3(key0), 1e-7));
assert(similarity3_1.equals(similarity3_result.atSimilarity3(key1), 1e-7));

%% SL4
sl4_0 = SL4.Expmap(linspace(0.001, 0.015, 15)');
sl4_1 = SL4.Expmap(linspace(-0.01, 0.02, 15)');
sl4_result = synchronizePair(sl4_0, sl4_1, @BetweenFactorSL4, ...
    @PriorFactorSL4, 15, @fastSyncSL4, key0, key1);
assert(sl4_0.equals(sl4_result.atSL4(key0), 1e-6));
assert(sl4_1.equals(sl4_result.atSL4(key1), 1e-6));

fprintf('FAST-Sync MATLAB example passed for all seven groups.\n');

function result = synchronizePair(value0, value1, betweenFactor, priorFactor, ...
    dimension, solver, key0, key1)
%SYNCHRONIZEPAIR Construct and solve one two-node synchronization graph.
model = gtsam.noiseModel.Diagonal.Sigmas(0.1 * ones(dimension, 1));
graph = gtsam.NonlinearFactorGraph;
graph.add(betweenFactor(key0, key1, value0.between(value1), model));
graph.add(priorFactor(key0, value0, model));
result = solver(graph);
end
