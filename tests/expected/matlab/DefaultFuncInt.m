function varargout = DefaultFuncInt(varargin)
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric')
        functions_wrapper(8, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'numeric')
        functions_wrapper(9, varargin{:});
      elseif length(varargin) == 0
        functions_wrapper(10, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncInt');
      end
end
