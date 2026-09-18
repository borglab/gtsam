function varargout = customFactorRegistry(operation, varargin)
% customFactorRegistry MATLAB-side storage for CustomFactor callbacks.
%
% C++ stores only a numeric callback ID. This registry owns:
%   1. the original MATLAB function handle
%   2. the MATLAB gtsam.CustomFactor object to pass back as `this`
%
% The split matters because the native factor can be copied and kept alive by
% optimizer-owned C++ objects, while MATLAB objects follow different lifetime
% rules. Keeping the authoritative callback state in one persistent registry
% avoids storing raw MATLAB handles in C++.
persistent callbacks nextId

if isempty(callbacks)
    callbacks = containers.Map('KeyType', 'uint64', 'ValueType', 'any');
    nextId = uint64(1);
end

switch operation
    case 'register'
        callback = varargin{1};
        if ~isa(callback, 'function_handle')
            error('CustomFactor callback must be a function handle');
        end
        % Allocate a stable ID before the native factor is constructed. The
        % MATLAB object itself is bound later, after the superclass constructor
        % returns and the final wrapper object exists.
        callbackId = nextId;
        nextId = nextId + uint64(1);
        callbacks(callbackId) = struct('callback', callback, 'factor', []);
        varargout{1} = callbackId;

    case 'bind'
        callbackId = uint64(varargin{1});
        entry = lookupEntry(callbacks, callbackId);
        % Store the user-visible MATLAB factor handle so invoke() can preserve
        % the callback signature errorFunc(this, values[, ...]).
        entry.factor = varargin{2};
        callbacks(callbackId) = entry;

    case 'invoke'
        callbackId = uint64(varargin{1});
        entry = lookupEntry(callbacks, callbackId);
        if isempty(entry.factor)
            error('CustomFactor callback %u is not bound to a factor', ...
                  callbackId);
        end
        % Forward all remaining arguments so C++ can request either:
        %   err = callback(this, values)
        % or:
        %   [err, H] = callback(this, values)
        if nargout > 1
            [varargout{1:nargout}] = entry.callback(entry.factor, varargin{2:end});
        else
            varargout{1} = entry.callback(entry.factor, varargin{2:end});
        end

    case 'remove'
        callbackId = uint64(varargin{1});
        if isKey(callbacks, callbackId)
            remove(callbacks, callbackId);
        end

    case 'count'
        varargout{1} = uint64(callbacks.Count);

    otherwise
        error('Unknown CustomFactor registry operation: %s', operation);
end

end

function entry = lookupEntry(callbacks, callbackId)
callbackId = uint64(callbackId);
if ~isKey(callbacks, callbackId)
    error('CustomFactor callback %u is not registered', callbackId);
end
entry = callbacks(callbackId);
end
