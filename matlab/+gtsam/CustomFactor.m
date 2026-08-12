classdef CustomFactor < gtsam.MatlabCustomFactor
  properties(Access = private)
    % Registry key for the MATLAB callback associated with this factor.
    callbackId = uint64(0)
  end

  methods
    function obj = CustomFactor(varargin)
      % Public constructor:
      %   CustomFactor(noiseModel, keys, errorFunc)
      %
      % Internal pointer constructor:
      %   CustomFactor(uint64_magic, ptr[, 'void'])
      %
      % The public path registers the MATLAB function handle first, then
      % passes only a numeric callback ID into the native wrapper. After the
      % superclass constructor returns, this MATLAB object binds itself into
      % the registry so future evaluations call:
      %   errorFunc(this, values)
      % or
      %   [error, H] = errorFunc(this, values)
      constructedArgs = {};
      callbackId = uint64(0);

      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && ...
          isa(varargin{1}, 'uint64') && ...
          varargin{1} == uint64(5139824614673773682)
        constructedArgs = varargin;
      else
        if nargin ~= 3
          error(['Arguments do not match any overload of gtsam.CustomFactor ', ...
                 'constructor']);
        end

        noiseModel = varargin{1};
        keys = gtsam.CustomFactor.normalizeKeys(varargin{2});
        callbackId = gtsam.customFactorRegistry('register', varargin{3});
        constructedArgs = {noiseModel, keys, callbackId};
      end

      obj = obj@gtsam.MatlabCustomFactor(constructedArgs{:});
      obj.callbackId = callbackId;
      if callbackId ~= 0
        gtsam.customFactorRegistry('bind', callbackId, obj);
      end
    end

    function delete(obj)
      for i = 1:numel(obj)
        if obj(i).callbackId ~= 0
          % Remove the MATLAB-side registry entry before the native handle is
          % torn down so repeated test runs do not leave stale callbacks behind.
          gtsam.customFactorRegistry('remove', obj(i).callbackId);
          obj(i).callbackId = uint64(0);
        end
        delete@gtsam.MatlabCustomFactor(obj(i));
      end
    end
  end

  methods(Static, Access = private)
    function keyVector = normalizeKeys(keys)
      % Accept the common MATLAB shorthand of numeric keys while preserving the
      % wrapped KeyVector path for callers that already use toolbox utilities.
      if isa(keys, 'gtsam.KeyVector')
        keyVector = keys;
        return
      end

      if isnumeric(keys)
        % Explicitly use KeyVector and push_back to avoid losing precision
        % with double-precision Vector utilities, which mangles Symbol keys.
        keyVector = gtsam.KeyVector();
        for i = 1:numel(keys)
          keyVector.push_back(uint64(keys(i)));
        end
        return
      end

      error(['CustomFactor keys must be either a numeric array or a ', ...
             'gtsam.KeyVector']);
    end
  end
end
