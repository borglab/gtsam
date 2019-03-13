function varargout = return_ptrs(varargin)
      if length(varargin) == 2 && isa(varargin{1},'Test') && isa(varargin{2},'Test')
        varargout{1} = geometry_wrapper(113, varargin{:});
      else
        error('Arguments do not match any overload of function return_ptrs');
      end
