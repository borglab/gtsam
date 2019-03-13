function varargout = vectorConfusion(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(92, varargin{:});
      else
        error('Arguments do not match any overload of function vectorConfusion');
      end
