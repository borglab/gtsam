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
    callbacks = containers.Map('KeyType', 'char', 'ValueType', 'any');
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
        callbacks(callbackKey(callbackId)) = struct('callback', callback, ...
                                                    'factor', []);
        varargout{1} = callbackId;

    case 'bind'
        callbackId = varargin{1};
        entry = lookupEntry(callbacks, callbackId);
        % Store the user-visible MATLAB factor handle so invoke() can preserve
        % the callback signature errorFunc(this, values[, ...]).
        entry.factor = varargin{2};
        callbacks(callbackKey(callbackId)) = entry;

    case 'invoke'
        callbackId = varargin{1};
        entry = lookupEntry(callbacks, callbackId);
        if isempty(entry.factor)
            error('CustomFactor callback %s is not bound to a factor', ...
                  callbackKey(callbackId));
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
        callbackId = varargin{1};
        key = callbackKey(callbackId);
        if isKey(callbacks, key)
            remove(callbacks, key);
        end

    case 'count'
        varargout{1} = uint64(callbacks.Count);

    otherwise
        error('Unknown CustomFactor registry operation: %s', operation);
end

end

function entry = lookupEntry(callbacks, callbackId)
key = callbackKey(callbackId);
if ~isKey(callbacks, key)
    error('CustomFactor callback %s is not registered', key);
end
entry = callbacks(key);
end

function key = callbackKey(callbackId)
key = char(string(uint64(callbackId)));
end
