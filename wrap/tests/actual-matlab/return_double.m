function varargout = return_double(varargin)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(100, varargin{:});
      else
        error('Arguments do not match any overload of function return_double');
      end
