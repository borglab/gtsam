function varargout = DefaultFuncObj(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyFormatter')
        functions_wrapper(14, varargin{:});
      elseif length(varargin) == 0
        functions_wrapper(15, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncObj');
      end
end
