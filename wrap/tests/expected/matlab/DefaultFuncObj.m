function varargout = DefaultFuncObj(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyFormatter')
        functions_wrapper(10, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncObj');
      end
