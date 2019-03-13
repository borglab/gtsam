function varargout = dim(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(87, varargin{:});
      else
        error('Arguments do not match any overload of function dim');
      end
