function varargout = return_string(varargin)
      if length(varargin) == 1 && isa(varargin{1},'string')
        varargout{1} = geometry_wrapper(101, varargin{:});
      else
        error('Arguments do not match any overload of function return_string');
      end
