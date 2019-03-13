function varargout = print(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(114, varargin{:});
      else
        error('Arguments do not match any overload of function print');
      end
