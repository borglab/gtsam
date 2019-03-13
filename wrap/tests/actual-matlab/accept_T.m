function varargout = accept_T(varargin)
      if length(varargin) == 1 && isa(varargin{1},'point2 const')
        varargout{1} = geometry_wrapper(119, varargin{:});
      else
        error('Arguments do not match any overload of function accept_T');
      end
