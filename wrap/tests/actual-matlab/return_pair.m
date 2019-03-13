function varargout = return_pair(varargin)
      if length(varargin) == 2 && isa(varargin{1},'vector') && isa(varargin{2},'matrix')
        varargout{1} = geometry_wrapper(96, varargin{:});
      else
        error('Arguments do not match any overload of function return_pair');
      end
