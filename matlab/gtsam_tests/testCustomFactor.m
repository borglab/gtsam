import gtsam.*;

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
initial = Values;
initial.insert(key, [0]);

baselineCallbacks = gtsam.customFactorRegistry('count');
model = noiseModel.Unit.Create(1);
factor = CustomFactor(model, key, @customFactorError);

EXPECT('registry create', gtsam.customFactorRegistry('count') == baselineCallbacks + 1);

errorVector = factor.unwhitenedError(initial);
EQUALITY('custom residual', [0] - target, errorVector, tol);

% Wrapped overloaded methods still need an explicit lambda in MATLAB.
expected_H = numericalDerivative.numericalDerivative11(@(v) factor.unwhitenedError(v), initial);
actual_H = factor.linearize(initial).jacobian();
EQUALITY('custom jacobian', expected_H, actual_H, 1e-5);

graph = NonlinearFactorGraph;
graph.add(factor);
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimize();

EQUALITY('custom factor optimize', target, result.atVector(key), 1e-6);
EXPECT('callback invoked', gtsamCustomFactorCalls > 0);
EXPECT('jacobians requested', gtsamCustomFactorJacobianCalls > 0);
EXPECT('wrapped inputs', gtsamCustomFactorSawWrappedInputs);

delete(result);
delete(optimizer);
delete(graph);
delete(factor);
delete(initial);
clear result optimizer graph factor initial;

EXPECT('registry cleanup', gtsam.customFactorRegistry('count') == baselineCallbacks);

factor = CustomFactor(model, [key], @customFactorError);
EXPECT('registry recreate', gtsam.customFactorRegistry('count') == baselineCallbacks + 1);
delete(factor);
clear factor;
EXPECT('registry final cleanup', gtsam.customFactorRegistry('count') == baselineCallbacks);

function varargout = customFactorError(this, values)
% Simple 1D residual used to verify both residual-only and Jacobian callback
% paths. The callback also checks that MATLAB receives wrapped toolbox objects,
% not raw numeric stand-ins.
global gtsamCustomFactorCalls;
global gtsamCustomFactorJacobianCalls;
global gtsamCustomFactorSawWrappedInputs;

gtsamCustomFactorCalls = gtsamCustomFactorCalls + 1;
gtsamCustomFactorSawWrappedInputs = isa(this, 'gtsam.CustomFactor') && ...
    isa(values, 'gtsam.Values');

keys = this.keys();
if keys.at(0) ~= 42
    error('CustomFactor:incorrectKey', 'Expected key 42, but got %d', keys.at(0));
end
current = values.atVector(keys.at(0));
residual = current - [3];

if nargout > 1
    gtsamCustomFactorJacobianCalls = gtsamCustomFactorJacobianCalls + 1;
    varargout{1} = residual;
    varargout{2} = {1};
else
    varargout{1} = residual;
end
end
