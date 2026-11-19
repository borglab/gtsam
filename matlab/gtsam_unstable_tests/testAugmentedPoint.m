%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Tests unstable augmented points, Values access, and QCQP bindings.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function testAugmentedPoint()
for dimension = [2, 3]
    testPoint(dimension);
    testAugmentedValues(dimension);
    testQcqpConversions(dimension);
end
end

function testPoint(dimension)
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
feval(sprintf('gtsam.insertAugmentedPoint%d', dimension), key, point, values);
read = @(key) feval(sprintf('gtsam.atAugmentedPoint%d', dimension), key, values);
feval(sprintf('gtsam.updateAugmentedPoint%d', dimension), ...
    key, pointType(coordinates + 1), values);
updated = read(key);
assert(isequal(updated.vector(), coordinates + 1));
feval(sprintf('gtsam.insertOrAssignAugmentedPoint%d', dimension), key, point, values);
feval(sprintf('gtsam.insertOrAssignAugmentedPoint%d', dimension), ...
    key + uint64(1), point, values);
clear point;
for repeat = 1:3
    point = read(key);
    other = read(key + uint64(1));
    assert(isequal(point.vector(), coordinates));
    assert(isequal(other.vector(), coordinates));
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
extractedPoint = feval(sprintf('gtsam.atAugmentedPoint%d', dimension), key, extracted);
assert(extractedPoint.equals(point));
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
