function varargout = DefaultFuncString(varargin)
      if length(varargin) == 2 && isa(varargin{1},'char') && isa(varargin{2},'char')
        functions_wrapper(9, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncString');
      end
