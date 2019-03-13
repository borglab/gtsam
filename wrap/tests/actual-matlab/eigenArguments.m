function varargout = eigenArguments(varargin)
      if length(varargin) == 2 && isa(varargin{1},'vector') && isa(varargin{2},'matrix')
        varargout{1} = geometry_wrapper(91, varargin{:});
      else
        error('Arguments do not match any overload of function eigenArguments');
      end
