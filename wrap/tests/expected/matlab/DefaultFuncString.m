function varargout = DefaultFuncString(varargin)
      if length(varargin) == 2 && isa(varargin{1},'char') && isa(varargin{2},'char')
        functions_wrapper(11, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'char')
        functions_wrapper(12, varargin{:});
      elseif length(varargin) == 0
        functions_wrapper(13, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncString');
      end
end
