function varargout = returnChar(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(88, varargin{:});
      else
        error('Arguments do not match any overload of function returnChar');
      end
