function varargout = create_ptrs(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(111, varargin{:});
      else
        error('Arguments do not match any overload of function create_ptrs');
      end
