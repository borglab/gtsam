function varargout = templatedMethodPoint2(varargin)
      if length(varargin) == 1 && isa(varargin{1},'point2 const')
        varargout{1} = geometry_wrapper(115, varargin{:});
      else
        error('Arguments do not match any overload of function templatedMethodPoint2');
      end
