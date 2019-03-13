function varargout = return_size_t(varargin)
      if length(varargin) == 1 && isa(varargin{1},'size_t')
        varargout{1} = geometry_wrapper(98, varargin{:});
      else
        error('Arguments do not match any overload of function return_size_t');
      end
