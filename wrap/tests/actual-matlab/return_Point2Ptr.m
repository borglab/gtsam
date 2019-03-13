function varargout = return_Point2Ptr(varargin)
      if length(varargin) == 1 && isa(varargin{1},'bool')
        varargout{1} = geometry_wrapper(110, varargin{:});
      else
        error('Arguments do not match any overload of function return_Point2Ptr');
      end
