function varargout = arg_EigenConstRef(varargin)
      if length(varargin) == 1 && isa(varargin{1},'matrix const')
        varargout{1} = geometry_wrapper(106, varargin{:});
      else
        error('Arguments do not match any overload of function arg_EigenConstRef');
      end
