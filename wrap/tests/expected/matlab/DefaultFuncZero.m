function varargout = DefaultFuncZero(varargin)
      if length(varargin) == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'logical')
        functions_wrapper(16, varargin{:});
      elseif length(varargin) == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'numeric')
        functions_wrapper(17, varargin{:});
      elseif length(varargin) == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double')
        functions_wrapper(18, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric')
        functions_wrapper(19, varargin{:});
      else
        error('Arguments do not match any overload of function DefaultFuncZero');
      end
end
