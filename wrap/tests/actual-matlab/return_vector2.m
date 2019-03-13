function varargout = return_vector2(varargin)
      if length(varargin) == 1 && isa(varargin{1},'vector')
        varargout{1} = geometry_wrapper(104, varargin{:});
      else
        error('Arguments do not match any overload of function return_vector2');
      end
