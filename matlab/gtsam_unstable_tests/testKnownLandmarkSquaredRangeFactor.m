%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Tests unstable squared-range factors with unstable points and stable graphs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function testKnownLandmarkSquaredRangeFactor()
for dimension = [2, 3]
    testFactorErrors(dimension);
    testQcqpFactorConversion(dimension);
end
end

function testFactorErrors(dimension)
% Both state types give matching residuals and reject invalid landmarks.
model = gtsam.noiseModel.Unit.Create(1);
coordinates = (1:dimension)';
point = feval(sprintf('gtsam.AugmentedPoint%d', dimension), coordinates);
factorType = str2func(sprintf( ...
    'gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint%d', dimension));
ordinaryType = str2func(sprintf( ...
    'gtsam.KnownLandmarkSquaredRangeFactor%d', dimension));
factor = factorType(0, zeros(dimension, 1), 1.0, model);
ordinary = ordinaryType(0, zeros(dimension, 1), 1.0, model);
residual = coordinates' * coordinates - 1;
assert(abs(factor.evaluateError(point) - residual) < 1e-12);
assert(abs(ordinary.evaluateError(coordinates) - residual) < 1e-12);
exact = factorType(0, zeros(dimension, 1), norm(coordinates), model);
assert(abs(exact.evaluateError(point)) < 1e-12);
values = gtsam.Values();
feval(sprintf('gtsam.insertAugmentedPoint%d', dimension), 0, point, values);
graph = gtsam.NonlinearFactorGraph();
graph.add(factor);
assert(abs(graph.error(values) - 0.5 * residual^2) < 1e-12);
for landmarkSize = [dimension - 1, dimension + 1]
    assertThrows(@() factorType(0, zeros(landmarkSize, 1), 1.0, model), ...
        'invalid landmark dimension');
    assertThrows(@() ordinaryType(0, zeros(landmarkSize, 1), 1.0, model), ...
        'invalid landmark dimension');
end
end

function testQcqpFactorConversion(dimension)
% QCQP construction requires an augmented state and a single column.
model = gtsam.noiseModel.Unit.Create(1);
graph = gtsam.NonlinearFactorGraph();
graph.add(feval(sprintf( ...
    'gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint%d', dimension), ...
    0, zeros(dimension, 1), 1.0, model));
problem = gtsam.QcqpProblem(graph);
assert(isa(problem, 'gtsam.QcqpProblem'));
assertThrows(@() gtsam.QcqpProblem(graph, 2), 'only supports D=1');
invalidGraph = gtsam.NonlinearFactorGraph();
invalidGraph.add(feval(sprintf( ...
    'gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint%d', dimension), ...
    0, zeros(dimension, 1), 1.0, gtsam.noiseModel.Isotropic.Sigma(2, 1.0)));
assertThrows(@() gtsam.QcqpProblem(invalidGraph), 'one-dimensional noise');
ordinaryGraph = gtsam.NonlinearFactorGraph();
ordinaryGraph.add(feval(sprintf( ...
    'gtsam.KnownLandmarkSquaredRangeFactor%d', dimension), ...
    0, zeros(dimension, 1), 1.0, model));
assertThrows(@() gtsam.QcqpProblem(ordinaryGraph), 'AugmentedPoint');
end

function assertThrows(action, expectedMessage)
% Ensure C++ validation failures become catchable MATLAB errors.
try
    action();
catch ME
    assert(contains(ME.message, expectedMessage), ...
        'Expected an error containing "%s"; received: %s', ...
        expectedMessage, ME.message);
    return;
end
error('Expected an error containing "%s", but none was thrown.', expectedMessage);
end
