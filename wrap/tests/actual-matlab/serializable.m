function varargout = serializable(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(93, varargin{:});
      else
        error('Arguments do not match any overload of function serializable');
      end
