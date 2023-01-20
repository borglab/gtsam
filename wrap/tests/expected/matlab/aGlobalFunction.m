function varargout = aGlobalFunction(varargin)
      if length(varargin) == 0
        varargout{1} = functions_wrapper(3, varargin{:});
      else
        error('Arguments do not match any overload of function aGlobalFunction');
      end
end
