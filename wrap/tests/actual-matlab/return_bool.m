function varargout = return_bool(varargin)
      if length(varargin) == 1 && isa(varargin{1},'bool')
        varargout{1} = geometry_wrapper(97, varargin{:});
      else
        error('Arguments do not match any overload of function return_bool');
      end
