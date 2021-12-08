function varargout = DefaultFuncInt(varargin)
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric')
        functions_wrapper(8, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncInt');
      end
