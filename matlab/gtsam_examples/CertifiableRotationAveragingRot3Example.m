%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Certifiable rotation averaging on SO(3).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

close all;
rng(42, 'twister');

%% Build noncommuting rotations with consecutive and loop-closure factors
numRotations = 5;
keys = zeros(numRotations, 1, 'uint64');
groundTruth = cell(numRotations, 1);
for index = 1:numRotations
    offset = index - 1;
    keys(index) = uint64(offset);
    groundTruth{index} = Rot3.RzRyRx( ...
        0.16 * offset, -0.10 * offset, 0.13 * offset);
end

edges = [
    1, 2;
    2, 3;
    3, 4;
    4, 5;
    5, 1;
    1, 3;
    2, 4;
    3, 5
];

graph = NonlinearFactorGraph;
for edge = 1:size(edges, 1)
    source = edges(edge, 1);
    target = edges(edge, 2);
    measurement = groundTruth{source}.between(groundTruth{target});
    graph.add(FrobeniusBetweenFactorRot3( ...
        keys(source), keys(target), measurement));
end

initial = Values;
initialRotations = cell(numRotations, 1);
for index = 1:numRotations
    perturbation = Rot3.Expmap(0.10 * randn(3, 1));
    perturbed = groundTruth{index}.compose(perturbation);
    initialRotations{index} = perturbed.matrix();
    % The QCQP variable associated with rotation R_i is X_i = R_i'.
    initial.insert(keys(index), initialRotations{index}');
end

fprintf('%d relative-rotation factors, %d variables\n', ...
    size(edges, 1), numRotations);

%% Run the Riemannian Staircase from pMin = 3
almParams = AugmentedLagrangianParams;
almParams.maxIterations = 100;
almParams.initialMuEq = 10.0;
almParams.muEqIncreaseRate = 2.0;
almParams.absoluteViolationTolerance = 1e-8;
almParams.relativeViolationTolerance = 1e-8;
almParams.absoluteCostTolerance = 1e-10;
almParams.relativeCostTolerance = 1e-10;

params = RiemannianStaircaseParams;
params.pMin = 3;
params.pMax = 5;
params.eta = 1e-3;
params.setAlmParams(almParams);

wallStart = tic;
result = RiemannianStaircaseOptimizer(graph, initial, params).optimize();
wallTime = toc(wallStart);

fprintf('Certified: %d\n', result.certified);
fprintf('Final rank: %d\n', result.finalRank);
fprintf('Certificate lower bound / minimum eigenvalue: %.3e\n', ...
    result.minEigenvalue);
fprintf('Solver time: %.3f s; example wall time: %.3f s\n', ...
    result.totalTime, wallTime);

if ~result.hasRoundedSolution()
    error('The staircase did not return a rounded solution.');
end

%% Correct the common determinant sign, project to SO(3), and fix the gauge
roundedValues = result.roundedValues();
roundedBlocks = cell(numRotations, 1);
negativeDeterminants = 0;
for index = 1:numRotations
    roundedBlocks{index} = roundedValues.atMatrix(keys(index));
    negativeDeterminants = negativeDeterminants + ...
        (det(roundedBlocks{index}) < 0);
end

% Apply one common reflection to every rounded block when needed.
if negativeDeterminants > floor(numRotations / 2)
    for index = 1:numRotations
        roundedBlocks{index}(:, end) = -roundedBlocks{index}(:, end);
    end
end

roundedRotations = cell(numRotations, 1);
truthMatrices = cell(numRotations, 1);
for index = 1:numRotations
    roundedRotations{index} = projectToSO(roundedBlocks{index}');
    truthMatrices{index} = groundTruth{index}.matrix();
end

initialAligned = gaugeAlign(initialRotations, truthMatrices);
roundedAligned = gaugeAlign(roundedRotations, truthMatrices);

initialError = zeros(numRotations, 1);
roundedError = zeros(numRotations, 1);
for index = 1:numRotations
    initialError(index) = rotationErrorDegrees( ...
        initialAligned{index}, truthMatrices{index});
    roundedError(index) = rotationErrorDegrees( ...
        roundedAligned{index}, truthMatrices{index});
end

fprintf('Maximum initial orientation error: %.3f deg\n', max(initialError));
fprintf('Maximum rounded orientation error: %.3e deg\n', max(roundedError));

%% Plot orientation error before and after optimization
indices = 0:(numRotations - 1);
figure('Name', 'Certifiable SO(3) rotation averaging');
bar(indices, [initialError, roundedError]);
xlabel('rotation index');
ylabel('geodesic orientation error (deg)');
title('Orientation error after gauge alignment');
grid on;
legend('initial', 'rounded', 'Location', 'best');

%% Plot per-level solver diagnostics
ranks = result.getRanksVisited();
costs = result.getCostPerLevel();
eigenvalues = result.getMinEigenvaluePerLevel();
nlpTimes = result.getNlpTimePerLevel();
verifyTimes = result.getVerifyTimePerLevel();
levels = 1:numel(ranks);

figure('Name', 'Certifiable SO(3) diagnostics');
tiledlayout(2, 2);

nexttile;
plot(levels, ranks, 'o-');
xlabel('staircase level');
ylabel('rank');
title('Rank');
grid on;

nexttile;
plot(ranks, costs, 'o-');
xlabel('staircase rank');
ylabel('cost');
title('Objective');
grid on;

nexttile;
plot(ranks, eigenvalues, 'o-');
hold on;
yline(-params.eta, '--r', '-\eta');
xlabel('staircase rank');
ylabel('eigenvalue / lower bound');
title('Certificate');
grid on;

nexttile;
bar(ranks, [nlpTimes, verifyTimes], 'stacked');
xlabel('staircase rank');
ylabel('seconds');
title('Runtime by level');
legend('local solve', 'verification', 'Location', 'best');
grid on;

%% Local matrix helpers
function rotation = projectToSO(matrix)
    [left, ~, right] = svd(matrix);
    correction = eye(size(matrix, 1));
    correction(end, end) = det(left * right');
    rotation = left * correction * right';
end

function aligned = gaugeAlign(rotations, reference)
    gauge = reference{1} * rotations{1}';
    aligned = cell(size(rotations));
    for index = 1:numel(rotations)
        aligned{index} = gauge * rotations{index};
    end
end

function error = rotationErrorDegrees(estimate, truth)
    relative = truth' * estimate;
    cosine = max(-1, min(1, (trace(relative) - 1) / 2));
    error = rad2deg(acos(cosine));
end
