function varargout = overloadedGlobalFunction(varargin)
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = functions_wrapper(4, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        varargout{1} = functions_wrapper(5, varargin{:});
      else
        error('Arguments do not match any overload of function overloadedGlobalFunction');
      end
end
