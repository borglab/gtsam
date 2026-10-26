function testGradients
% Verify the shared Gradients API, keyed tangent vectors, and factor residual blocks.
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation.
% All Rights Reserved. See LICENSE for the license information.

first = gtsam.symbol('x', 1);
second = gtsam.symbol('x', 2);
assert(first > uint64(2^53), 'Use a key that cannot round-trip through double.');
keys = [first, second];
priorNoise = gtsam.noiseModel.Gaussian.Covariance([0.16, 0.03; 0.03, 0.09]);
betweenNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3; 0.5]);
finalNoise = gtsam.noiseModel.Isotropic.Sigma(2, 0.7);
graph = gtsam.NonlinearFactorGraph;
graph.add(gtsam.PriorFactorPoint2(first, [0; 0], priorNoise));
graph.resize(2); % Preserve a null factor slot in the factor residual order.
graph.add(gtsam.BetweenFactorPoint2(first, second, [1; 0.2], betweenNoise));
graph.add(gtsam.PriorFactorPoint2(second, [1.1; 0.4], finalNoise));
solution = gtsam.Values;
solution.insert(first, [0.1; -0.2]);
solution.insert(second, [1.2; 0.5]);
prior = priorNoise.Whiten(eye(2));
between = betweenNoise.Whiten(eye(2));
final = finalNoise.Whiten(eye(2));
jacobian = [prior, zeros(2); -between, between; zeros(2), final];
seed = gtsam.VectorValues;
seed.insert(second, [1.5; -0.4]);
direction = {[0.2; -0.1], [0.4; 0.3], [-0.5; 0.6]};
ordering = gtsam.Ordering;
ordering.push_back(second);
ordering.push_back(first);

% Explicit ordinary JacobianFactors avoid projected/custom linearization rows.
linearized = gtsam.GaussianFactorGraph;
linearized.add(first, prior, zeros(2, 1));
linearized.add(first, -between, second, between, zeros(2, 1));
linearized.add(second, final, zeros(2, 1));
tree = linearized.eliminateMultifrontal(ordering);
imported = gtsam.Gradients(linearized, tree);
lowerSolution = tree.backSubstituteTranspose(seed);
treeAdjoint = tree.backSubstitute(lowerSolution);
assert(treeAdjoint.equals(imported.adjoint(seed), 1e-11));
displayText = evalc('disp(imported)');
assert(contains(displayText, '6 factor residual'));
constructors = {gtsam.Gradients(graph, solution), gtsam.Gradients(linearized), imported};
hasEmptySlot = [true, false, false];
for factorization = [gtsam.Gradients.Factorization.CHOLESKY, gtsam.Gradients.Factorization.QR]
    constructors = [constructors, { ...
        gtsam.Gradients(graph, solution, factorization), ...
        gtsam.Gradients(graph, solution, ordering, factorization), ...
        gtsam.Gradients(linearized, factorization), ...
        gtsam.Gradients(linearized, ordering, factorization)}]; %#ok<AGROW>
    hasEmptySlot = [hasEmptySlot, true, true, false, false]; %#ok<AGROW>
end
for i = 1:numel(constructors)
    checkQueries(constructors{i}, keys, jacobian, seed, direction, hasEmptySlot(i));
end

% Parameter identity survives copies; equal display labels do not alias.
tag = gtsam.ParameterTag('calibration');
copiedTag = gtsam.ParameterTag(tag);
assert(strcmp(tag.name(), 'calibration'));
assert(tag.equals(copiedTag));
assert(tag.hash() == copiedTag.hash());
assert(~tag.equals(gtsam.ParameterTag('calibration')));
anonymousTag = gtsam.ParameterTag;
assert(~anonymousTag.equals(gtsam.ParameterTag));
clear tag
assert(strcmp(copiedTag.name(), 'calibration'));
for repeat = 1:3
    % Ordinary factors declare no parameter dependencies.
    absent = constructors{1}.parameterGradient(seed, copiedTag);
    assert(isempty(absent));
    priorFactor = gtsam.PriorFactorPoint2(first, [0; 0], priorNoise);
    contribution = priorFactor.parameterVjp(solution, copiedTag, [1; 2]);
    assert(isempty(contribution));
end
assertThrows(@() imported.parameterGradient(seed, copiedTag));

% Owned conditionals survive source containers; shared numerical data is unchanged.
saved = imported.adjoint(seed);
savedFactorCotangents = imported.pullback(seed);
graph.remove(0);
clear graph solution linearized tree
for repeat = 1:3
    checkQueries(imported, keys, jacobian, seed, direction, false);
    checkQueries(constructors{1}, keys, jacobian, seed, direction, true);
end
again = imported.adjoint(seed);
assert(norm(saved.at(second) - again.at(second)) < 1e-12);
zero = imported.pullback(gtsam.VectorValues);
assert(gtsam.dot(zero, zero) == 0);
assert(imported.pushforward(zero).norm() == 0);
assertThrows(@() imported.pushforward(gtsam.Errors));
assertThrows(@() imported.pushforward(makeErrors({zeros(1, 1), zeros(2, 1), zeros(2, 1)})));
assertThrows(@() imported.pushforward(makeErrors({nan(2, 1), zeros(2, 1), zeros(2, 1)})));
assertThrows(@() constructors{1}.pushforward(makeErrors({zeros(2, 1), 1, zeros(2, 1), zeros(2, 1)})));
unknown = gtsam.VectorValues;
unknown.insert(gtsam.symbol('z', 99), [1; 0]);
assertThrows(@() imported.adjoint(unknown));

% Empty graphs preserve empty vector and container conversions in MATLAB.
empty = gtsam.Gradients(gtsam.GaussianFactorGraph);
assert(empty.adjoint(gtsam.VectorValues).size() == 0);
assert(empty.pushforward(gtsam.Errors).size() == 0);
assert(empty.pullback(gtsam.VectorValues).empty());
clear constructors imported
retained = errorBlocks(savedFactorCotangents);
expected = -jacobian * ((jacobian' * jacobian) \ [0; 0; 1.5; -0.4]);
assert(norm(vertcat(retained{:}) - expected) < 1e-11);
checkTreeSolves;
disp('Gradients MATLAB wrapper tests passed.');
end

function checkTreeSolves
% A known whitened triangular matrix checks both passes independently.
keys = [gtsam.symbol('x', 30), gtsam.symbol('x', 2), ...
        gtsam.symbol('x', 21), gtsam.symbol('x', 7)];
diagonal = [2; 3; 4; 5];
upper = [-1; 0.5; -0.7];
sigmas = [0.4; 0.7; 0.3; 1.2];
matrix = (diag(diagonal) + diag(upper, 1)) ./ sigmas;
graph = gtsam.GaussianFactorGraph;
ordering = gtsam.Ordering;
for i = 1:4
    ordering.push_back(keys(i));
    noise = gtsam.noiseModel.Isotropic.Sigma(1, sigmas(i));
    if i < 4
        factor = gtsam.JacobianFactor( ...
            keys(i), diagonal(i), keys(i + 1), upper(i), 9 + i, noise);
    else
        factor = gtsam.JacobianFactor(keys(i), diagonal(i), 13, noise);
    end
    graph.add(factor);
end
tree = graph.eliminateMultifrontal(ordering);
assert(tree.size() > 1);
denseRhs = [1.25; 0; -0.75; 0.5];
rhs = gtsam.VectorValues;
for i = [1, 3, 4]
    rhs.insert(keys(i), denseRhs(i));
end
upperSolution = tree.backSubstitute(rhs);
lowerSolution = tree.backSubstituteTranspose(rhs);
normalSolution = tree.backSubstitute(lowerSolution);
upperVector = zeros(4, 1);
lowerVector = zeros(4, 1);
normalVector = zeros(4, 1);
for i = 1:4
    upperVector(i) = upperSolution.at(keys(i));
    lowerVector(i) = lowerSolution.at(keys(i));
    normalVector(i) = normalSolution.at(keys(i));
end
assert(norm(upperVector - (matrix \ denseRhs)) < 1e-11);
assert(norm(lowerVector - (matrix' \ denseRhs)) < 1e-11);
assert(norm(normalVector - ((matrix' * matrix) \ denseRhs)) < 1e-11);
assert(rhs.size() == 3 && ~rhs.exists(keys(2)));
zeroUpper = tree.backSubstitute(gtsam.VectorValues);
zeroLower = tree.backSubstituteTranspose(gtsam.VectorValues);
assert(zeroUpper.size() == 4 && zeroUpper.norm() == 0);
assert(zeroLower.size() == 4 && zeroLower.norm() == 0);
empty = gtsam.GaussianBayesTree;
assert(empty.backSubstitute(gtsam.VectorValues).size() == 0);
assert(empty.backSubstituteTranspose(gtsam.VectorValues).size() == 0);
end

function checkQueries(gradients, keys, jacobian, seed, direction, hasEmptySlot)
% Factor blocks agree with independent dense sign and whitening references.
hessian = jacobian' * jacobian;
denseSeed = [0; 0; 1.5; -0.4];
expectedAdjoint = hessian \ denseSeed;
expectedForward = -(hessian \ (jacobian' * vertcat(direction{:})));
if hasEmptySlot
    direction = {direction{1}, zeros(0, 1), direction{2}, direction{3}};
end
factorDirection = makeErrors(direction);
adjoint = gradients.adjoint(seed);
forward = gradients.pushforward(factorDirection);
reverse = gradients.pullback(seed);
assert(norm([adjoint.at(keys(1)); adjoint.at(keys(2))] - expectedAdjoint) < 1e-11);
assert(norm([forward.at(keys(1)); forward.at(keys(2))] - expectedForward) < 1e-11);
blocks = errorBlocks(reverse);
assert(norm(vertcat(blocks{:}) + jacobian * expectedAdjoint) < 1e-11);
assert(abs(denseSeed' * expectedForward - gtsam.dot(reverse, factorDirection)) < 1e-11);
assert(reverse.size() == numel(direction));
assert(isequal(cellfun(@numel, blocks), cellfun(@numel, direction)));
end

function result = makeErrors(blocks)
% Errors retains one vector per factor, including empty entries.
result = gtsam.Errors;
for i = 1:numel(blocks)
    result.push_back(blocks{i});
end
end

function blocks = errorBlocks(errors)
% Traverse a copy so the returned factor-associated vectors remain reusable.
remaining = gtsam.Errors(errors);
blocks = cell(1, remaining.size());
for i = 1:numel(blocks)
    blocks{i} = remaining.front();
    remaining.pop_front();
end
assert(remaining.empty());
end

function assertThrows(operation)
% A wrapper must propagate validation failures, not silently coerce input.
threw = false;
try
    operation();
catch
    threw = true;
end
assert(threw, 'Expected Gradients to reject the invalid query.');
end
