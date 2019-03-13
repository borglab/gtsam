function varargout = return_matrix2(varargin)
      if length(varargin) == 1 && isa(varargin{1},'matrix')
        varargout{1} = geometry_wrapper(105, varargin{:});
      else
        error('Arguments do not match any overload of function return_matrix2');
      end
