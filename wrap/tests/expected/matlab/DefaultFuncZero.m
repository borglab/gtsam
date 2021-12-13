function varargout = DefaultFuncZero(varargin)
      if length(varargin) == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'logical') && isa(varargin{5},'logical')
        functions_wrapper(11, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncZero');
      end
