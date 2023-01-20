function varargout = TemplatedFunctionRot3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Rot3')
        functions_wrapper(25, varargin{:});
      else
        error('Arguments do not match any overload of function TemplatedFunctionRot3');
      end
end
