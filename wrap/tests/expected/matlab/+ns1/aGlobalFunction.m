function varargout = aGlobalFunction(varargin)
      if length(varargin) == 0
        varargout{1} = namespaces_wrapper(6, varargin{:});
      else
        error('Arguments do not match any overload of function aGlobalFunction');
      end
end
