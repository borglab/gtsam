function varargout = return_field(varargin)
      if length(varargin) == 1 && isa(varargin{1},'test const')
        varargout{1} = geometry_wrapper(107, varargin{:});
      else
        error('Arguments do not match any overload of function return_field');
      end
