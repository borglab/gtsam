%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Checks for results of functions in utilities namespace
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create keys for variables
x1 = gtsam.symbol('x',1); x2 = gtsam.symbol('x',2); x3 = gtsam.symbol('x',3);

actual = gtsam.utilities.createKeyList([1;2;3]);
gtsam.CHECK('KeyList', isa(actual,'gtsam.KeyList'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.front==1', actual.front==1);

actual = gtsam.utilities.createKeyList('x',[1;2;3]);
gtsam.CHECK('KeyList', isa(actual,'gtsam.KeyList'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.front==x1', actual.front==x1);

actual = gtsam.utilities.createKeyVector([1;2;3]);
gtsam.CHECK('KeyVector', isa(actual,'gtsam.KeyVector'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.at(0)==1', actual.at(0)==1);

actual = gtsam.utilities.createKeyVector('x',[1;2;3]);
gtsam.CHECK('KeyVector', isa(actual,'gtsam.KeyVector'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.at(0)==x1', actual.at(0)==x1);

actual = gtsam.utilities.createKeySet([1;2;3]);
gtsam.CHECK('KeySet', isa(actual,'gtsam.KeySet'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.count(1)', actual.count(1));

actual = gtsam.utilities.createKeySet('x',[1;2;3]);
gtsam.CHECK('KeySet', isa(actual,'gtsam.KeySet'));
gtsam.CHECK('size==3', actual.size==3);
gtsam.CHECK('actual.count(x1)', actual.count(x1));

% test extractVectors
values = gtsam.Values();
values.insert(gtsam.symbol('x', 0), (1:6)');
values.insert(gtsam.symbol('x', 1), (7:12)');
values.insert(gtsam.symbol('x', 2), (13:18)');
values.insert(gtsam.symbol('x', 7), gtsam.Pose3());
actual = gtsam.utilities.extractVectors(values, 'x');
expected = reshape(1:18, 6, 3)';
gtsam.CHECK('extractVectors', all(actual == expected, 'all'));

% test batch measurement matrix wrappers
tol = 1e-9;
pixels = [20 30; 20 30];
indices = [0; 1];
batchValues = gtsam.Values();
camera = gtsam.PinholeCameraCal3_S2();
gtsam.utilities.insertBackprojections(batchValues, camera, indices, pixels, 10);
gtsam.CHECK('insertBackprojections matrix', ...
    norm(batchValues.atPoint3(0) - gtsam.Point3(200, 200, 10)) < tol);

graph = gtsam.NonlinearFactorGraph();
model = gtsam.noiseModel.Isotropic.Sigma(2, 0.1);
gtsam.utilities.insertProjectionFactors( ...
    graph, 0, indices, pixels, model, gtsam.Cal3_S2());
gtsam.CHECK('insertProjectionFactors matrix', graph.size == 2);
