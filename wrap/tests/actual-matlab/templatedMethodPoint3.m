function varargout = templatedMethodPoint3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'point3 const')
        varargout{1} = geometry_wrapper(116, varargin{:});
      else
        error('Arguments do not match any overload of function templatedMethodPoint3');
      end
