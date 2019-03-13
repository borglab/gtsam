function varargout = return_matrix1(varargin)
      if length(varargin) == 1 && isa(varargin{1},'matrix')
        varargout{1} = geometry_wrapper(103, varargin{:});
      else
        error('Arguments do not match any overload of function return_matrix1');
      end
