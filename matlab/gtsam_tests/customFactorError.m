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
