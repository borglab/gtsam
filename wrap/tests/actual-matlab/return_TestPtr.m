function varargout = return_TestPtr(varargin)
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = geometry_wrapper(108, varargin{:});
      else
        error('Arguments do not match any overload of function return_TestPtr');
      end
