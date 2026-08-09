%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Certifiable rotation averaging on SO(2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

close all;
rng(42, 'twister');

%% Build a deterministic rotation ring
numRotations = 8;
delta = 2 * pi / numRotations;
keys = zeros(numRotations, 1, 'uint64');
groundTruth = cell(numRotations, 1);

for index = 1:numRotations
    keys(index) = uint64(index - 1);
    groundTruth{index} = Rot2((index - 1) * delta);
end

graph = NonlinearFactorGraph;
initial = Values;
initialRotations = cell(numRotations, 1);

for index = 1:numRotations
    nextIndex = mod(index, numRotations) + 1;
    measurement = groundTruth{index}.between(groundTruth{nextIndex});
    graph.add(FrobeniusBetweenFactorRot2( ...
        keys(index), keys(nextIndex), measurement));

    angle = groundTruth{index}.theta() + 0.22 * randn;
    initialRotations{index} = Rot2(angle).matrix();
    % The QCQP variable associated with rotation R_i is X_i = R_i'.
    initial.insert(keys(index), initialRotations{index}');
end

fprintf('%d relative-rotation factors, %d variables\n', ...
    numRotations, numRotations);

%% Run the Riemannian Staircase from pMin = 2
almParams = AugmentedLagrangianParams;
almParams.maxIterations = 100;
almParams.initialMuEq = 10.0;
almParams.muEqIncreaseRate = 2.0;
almParams.absoluteViolationTolerance = 1e-8;
almParams.relativeViolationTolerance = 1e-8;
almParams.absoluteCostTolerance = 1e-10;
almParams.relativeCostTolerance = 1e-10;

params = RiemannianStaircaseParams;
params.pMin = 2;
params.pMax = 4;
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

%% Correct the common determinant sign, project to SO(2), and fix the gauge
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

truthAngles = zeros(numRotations, 1);
initialAngles = zeros(numRotations, 1);
roundedAngles = zeros(numRotations, 1);
for index = 1:numRotations
    truthAngles(index) = angleOf(truthMatrices{index});
    initialAngles(index) = angleOf(initialAligned{index});
    roundedAngles(index) = angleOf(roundedAligned{index});
end
truthAngles = unwrap(truthAngles);
initialAngles = truthAngles + wrapAngle(initialAngles - truthAngles);
roundedAngles = truthAngles + wrapAngle(roundedAngles - truthAngles);

initialError = abs(rad2deg(initialAngles - truthAngles));
roundedError = abs(rad2deg(roundedAngles - truthAngles));
fprintf('Maximum initial error: %.3f deg\n', max(initialError));
fprintf('Maximum rounded error: %.3e deg\n', max(roundedError));

%% Plot rotations and gauge-aligned errors
indices = 0:(numRotations - 1);
figure('Name', 'Certifiable SO(2) rotation averaging');
tiledlayout(1, 2);

nexttile;
plot(indices, rad2deg(truthAngles), 'o-', 'DisplayName', 'ground truth');
hold on;
plot(indices, rad2deg(initialAngles), 's--', 'DisplayName', 'initial');
plot(indices, rad2deg(roundedAngles), 'x-', 'DisplayName', 'rounded');
xlabel('rotation index');
ylabel('gauge-aligned angle (deg)');
title('Rotations');
grid on;
legend('Location', 'best');

nexttile;
bar(indices, [initialError, roundedError]);
xlabel('rotation index');
ylabel('absolute angular error (deg)');
title('Error after gauge alignment');
grid on;
legend('initial', 'rounded', 'Location', 'best');

%% Plot per-level solver diagnostics
ranks = result.getRanksVisited();
costs = result.getCostPerLevel();
eigenvalues = result.getMinEigenvaluePerLevel();
nlpTimes = result.getNlpTimePerLevel();
verifyTimes = result.getVerifyTimePerLevel();
levels = 1:numel(ranks);

figure('Name', 'Certifiable SO(2) diagnostics');
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

function angle = angleOf(rotation)
    angle = atan2(rotation(2, 1), rotation(1, 1));
end

function angles = wrapAngle(angles)
    angles = mod(angles + pi, 2 * pi) - pi;
end
