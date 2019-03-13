function varargout = norm(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(94, varargin{:});
      else
        error('Arguments do not match any overload of function norm');
      end
