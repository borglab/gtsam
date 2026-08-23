% Regression coverage for the MATLAB CustomFactor bridge. This test checks the
% end-to-end path:
%   MATLAB callback registration -> native factor evaluation -> optimizer use
% and then verifies the registry is cleaned up when MATLAB objects are deleted.

global gtsamCustomFactorCalls;
global gtsamCustomFactorJacobianCalls;
global gtsamCustomFactorSawWrappedInputs;

gtsamCustomFactorCalls = 0;
gtsamCustomFactorJacobianCalls = 0;
gtsamCustomFactorSawWrappedInputs = false;

tol = 1e-9;
key = 42;
target = [3];
initial = gtsam.Values;
initial.insert(key, [0]);

baselineCallbacks = gtsam.customFactorRegistry('count');
model = gtsam.noiseModel.Unit.Create(1);
factor = gtsam.CustomFactor(model, key, @customFactorError);

gtsam.EXPECT('registry create', gtsam.customFactorRegistry('count') == baselineCallbacks + 1);

errorVector = factor.unwhitenedError(initial);
gtsam.EQUALITY('custom residual', [0] - target, errorVector, tol);

% Wrapped overloaded methods still need an explicit lambda in MATLAB.
expected_H = gtsam.numericalDerivative.numericalDerivative11( ...
    @(v) factor.unwhitenedError(v), initial);
actual_H = factor.linearize(initial).jacobian();
gtsam.EQUALITY('custom jacobian', expected_H, actual_H, 1e-5);

graph = gtsam.NonlinearFactorGraph;
graph.add(factor);
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimize();

gtsam.EQUALITY('custom factor optimize', target, result.atVector(key), 1e-6);
gtsam.EXPECT('callback invoked', gtsamCustomFactorCalls > 0);
gtsam.EXPECT('jacobians requested', gtsamCustomFactorJacobianCalls > 0);
gtsam.EXPECT('wrapped inputs', gtsamCustomFactorSawWrappedInputs);

delete(result);
delete(optimizer);
delete(graph);
delete(factor);
delete(initial);
clear result optimizer graph factor initial;

gtsam.EXPECT('registry cleanup', gtsam.customFactorRegistry('count') == baselineCallbacks);

factor = gtsam.CustomFactor(model, [key], @customFactorError);
gtsam.EXPECT('registry recreate', gtsam.customFactorRegistry('count') == baselineCallbacks + 1);
delete(factor);
clear factor;
gtsam.EXPECT('registry final cleanup', gtsam.customFactorRegistry('count') == baselineCallbacks);
