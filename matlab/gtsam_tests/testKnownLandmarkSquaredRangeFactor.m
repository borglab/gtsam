%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Tests augmented points, squared-range factors, and QCQP bindings.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function testKnownLandmarkSquaredRangeFactor()
for dimension = [2, 3]
    testAugmentedPoint(dimension);
    testAugmentedValues(dimension);
    testFactorErrors(dimension);
    testQcqpConversions(dimension);
    testQcqpFactorConversion(dimension);
end
end

function testAugmentedPoint(dimension)
% Point methods marshal coordinates and validate input dimensions.
pointType = str2func(sprintf('gtsam.AugmentedPoint%d', dimension));
coordinates = (1:dimension)';
point = pointType(coordinates);
assert(isequal(point.vector(), coordinates));
origin = pointType();
identity = feval(sprintf('gtsam.AugmentedPoint%d.Identity', dimension));
assert(isequal(origin.vector(), zeros(dimension, 1)));
assert(isequal(identity.vector(), zeros(dimension, 1)));
assert(point.dim() == dimension);
assert(abs(point.norm() - norm(coordinates)) < 1e-12);
assert(abs(point.dot(point) - coordinates' * coordinates) < 1e-12);
assert(point.equals(pointType(coordinates)));
assert(~point.equals(pointType(coordinates + 1)));
for pointSize = [dimension - 1, dimension + 1]
    assertThrows(@() pointType(zeros(pointSize, 1)), 'invalid point dimension');
end
end

function testAugmentedValues(dimension)
% Typed Values access preserves coordinates, lifetimes, and 64-bit keys.
pointType = str2func(sprintf('gtsam.AugmentedPoint%d', dimension));
coordinates = (1:dimension)';
key = bitshift(uint64(1), 40) + uint64(dimension);
point = pointType(coordinates);
values = gtsam.Values();
values.insert(key, point);
read = @(key) feval(sprintf('atAugmentedPoint%d', dimension), values, key);
values.update(key, pointType(coordinates + 1));
updated = read(key);
assert(isequal(updated.vector(), coordinates + 1));
values.insert_or_assign(key, point);
values.insert_or_assign(key + uint64(1), point);
clear point;
for repeat = 1:3
    point = read(key);
    other = read(key + uint64(1));
    assert(isequal(point.vector(), coordinates));
    assert(isequal(other.vector(), coordinates));
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
values.insert(0, point);
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

function testQcqpConversions(dimension)
% Lift, recovery, insertion, and extraction work for both dimensions.
coordinates = (1:dimension)';
point = feval(sprintf('gtsam.AugmentedPoint%d', dimension), coordinates);
key = bitshift(uint64(1), 40) + uint64(dimension);
lift = feval(sprintf('gtsam.qcqpValue%d', dimension), point);
expected = [1; coordinates; coordinates' * coordinates];
assert(isequal(lift, expected));
recover = str2func(sprintf('gtsam.fromQcqpValue%d', dimension));
recovered = recover(lift);
negative = recover(-lift);
assert(recovered.equals(point));
assert(negative.equals(point));
assertThrows(@() recover(zeros(dimension + 1, 1)), ...
    sprintf('requires a %d-by-1', dimension + 2));
assertThrows(@() recover(zeros(dimension + 2, 1)), ...
    'nonzero homogenization coordinate');
lifted = gtsam.Values();
insert = str2func(sprintf('gtsam.insertQcqpValue%d', dimension));
insert(key, point, lifted);
insert(key, point, lifted);
assert(isequal(lifted.atMatrix(key), expected));
extracted = feval(sprintf('gtsam.extractQcqpValues%d', dimension), lifted);
extractedPoint = feval(sprintf('atAugmentedPoint%d', dimension), extracted, key);
assert(extractedPoint.equals(point));
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
