function testWnoaAugmentedPoint()
% Regression tests for unstable AugmentedPoint3 WNOA MATLAB bindings.
% Run with the stable and unstable MATLAB wrapper directories on the path.

assert(exist('gtsam_wrapper', 'file') == 3, ...
    'Add the build-x64/wrap/gtsam_mex directory to the MATLAB path.');
assert(exist('gtsam_unstable_wrapper', 'file') == 3, ...
    'Add the build-x64/wrap/gtsam_unstable_mex directory to the MATLAB path.');

testMotion();
testInterpolatedRange();
testRangeOnlyOptimization();
fprintf('WNOA AugmentedPoint3 MATLAB tests passed (3 tests).\n');
end

function testMotion()
left = gtsam.StateData(uint64(1), uint64(2), 0.0);
right = gtsam.StateData(uint64(3), uint64(4), 0.2);
factor = gtsam.WnoaMotionFactorAugmentedPoint3(left, right, ones(3, 1));

values = gtsam.Values();
p = [1; -2; 3];
v = [0.5; -0.25; 0.75];
insertState(values, uint64(1), uint64(2), p, v);
insertState(values, uint64(3), uint64(4), p + 0.2 * v, v);
assert(abs(factor.error(values)) < 1e-9, 'Motion residual is not zero.');
fprintf('  Motion residual: passed\n');
end

function testInterpolatedRange()
left = gtsam.StateData(uint64(1), uint64(2), 0.0);
mid = gtsam.StateData(uint64(5), uint64(6), 0.5);
right = gtsam.StateData(uint64(3), uint64(4), 1.0);
noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1);
inner = gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint3( ...
    uint64(5), [2; 0; 0], 1.5, noise);
factor = gtsam.MatlabWnoaInterpFactorAugmentedPoint3( ...
    inner, left, mid, right, 0.1 * ones(3, 1), true, true);

kv = factor.keys();
keys = zeros(1, double(kv.size()), 'uint64');
for i = 0:numel(keys)-1
    keys(i+1) = kv.at(i);
end
assert(isequal(keys, uint64([1, 2, 3, 4])), ...
    'Interpolated factor uses unexpected optimization keys.');
assert(factor.dim() == 1, 'Squared-range factor should have one residual.');

values = gtsam.Values();
insertState(values, uint64(1), uint64(2), [1; 0; 0], [1; 0; 0]);
insertState(values, uint64(3), uint64(4), [2; 0; 0], [1; 0; 0]);
expected = 0.5 * ((0.5^2 - 1.5^2) / 0.1)^2;
assert(abs(factor.error(values) - expected) < 1e-7, ...
    'Interpolated squared-range error does not match the expected value.');
fprintf('  Interpolated range residual and keys: passed\n');
end

function testRangeOnlyOptimization()
p0 = [0.6; -0.2; 0.7];
v = [0.35; 0.10; -0.15];
q = 0.2 * ones(3, 1);
landmarks = [0, 3, 0, 0; ...
             0, 0, 3, 0; ...
             0, 0, 0, 3];

states = cell(1, 3);
for i = 1:3
    states{i} = gtsam.StateData(uint64(2*i - 1), uint64(2*i), i - 1);
end

graph = gtsam.NonlinearFactorGraph();
for i = 1:2
    graph.add(gtsam.WnoaMotionFactorAugmentedPoint3( ...
        states{i}, states{i+1}, q));
end

rangeNoise = gtsam.noiseModel.Isotropic.Sigma(1, 0.02);
% Anchor the first knot with measured ranges, without an AugmentedPoint prior.
for j = 1:size(landmarks, 2)
    landmark = landmarks(:, j);
    measured = norm(p0 - landmark);
    graph.add(gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint3( ...
        uint64(1), landmark, measured, rangeNoise));
end

% Measurements are between knots; only the knot states are optimized.
times = [0.25, 0.75, 1.25, 1.75];
for i = 1:numel(times)
    t = times(i);
    mid = gtsam.StateData(uint64(100 + i), uint64(200 + i), t);
    if t < 1
        left = states{1};
        right = states{2};
    else
        left = states{2};
        right = states{3};
    end
    position = p0 + t * v;
    for j = 1:size(landmarks, 2)
        landmark = landmarks(:, j);
        measured = norm(position - landmark);
        inner = gtsam.KnownLandmarkSquaredRangeFactorAugmentedPoint3( ...
            uint64(100 + i), landmark, measured, rangeNoise);
        graph.add(gtsam.MatlabWnoaInterpFactorAugmentedPoint3( ...
            inner, left, mid, right, q, true, true));
    end
end

initial = gtsam.Values();
for i = 1:3
    position = p0 + (i - 1) * v + [0.12; -0.08; 0.10];
    velocity = v + [-0.08; 0.06; 0.10];
    insertState(initial, uint64(2*i - 1), uint64(2*i), ...
        position, velocity);
end

initialError = graph.error(initial);
result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize();
finalError = graph.error(result);
assert(finalError < initialError, 'Optimization failed to reduce error.');
assert(finalError < 1e-4, 'Optimization did not recover the trajectory.');
assert(result.size() == 6, 'Unexpected number of optimized states.');

for i = 1:3
    position = gtsam.atAugmentedPoint3(uint64(2*i - 1), result).vector();
    velocity = result.atPoint3(uint64(2*i));
    assert(norm(position - (p0 + (i-1)*v)) < 1e-2, ...
        'Recovered knot position is inaccurate.');
    assert(norm(velocity - v) < 1e-2, ...
        'Recovered knot velocity is inaccurate.');
end
fprintf('  Range-only optimization: passed (final error %.3g)\n', finalError);
end

function insertState(values, positionKey, velocityKey, position, velocity)
gtsam.insertAugmentedPoint3(positionKey, ...
    gtsam.AugmentedPoint3(position), values);
values.insertPoint3(velocityKey, velocity);
end
