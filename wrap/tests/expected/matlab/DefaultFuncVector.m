function varargout = DefaultFuncVector(varargin)
      if length(varargin) == 2 && isa(varargin{1},'std.vectornumeric') && isa(varargin{2},'std.vectorchar')
        functions_wrapper(12, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncVector');
      end
