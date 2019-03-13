function varargout = return_int(varargin)
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = geometry_wrapper(99, varargin{:});
      else
        error('Arguments do not match any overload of function return_int');
      end
