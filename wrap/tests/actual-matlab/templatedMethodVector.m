function varargout = templatedMethodVector(varargin)
      if length(varargin) == 1 && isa(varargin{1},'vector const')
        varargout{1} = geometry_wrapper(117, varargin{:});
      else
        error('Arguments do not match any overload of function templatedMethodVector');
      end
