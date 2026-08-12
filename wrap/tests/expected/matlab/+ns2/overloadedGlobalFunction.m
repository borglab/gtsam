function varargout = overloadedGlobalFunction(varargin)
      if length(varargin) == 1 && isa(varargin{1},'ns1.ClassA')
        varargout{1} = namespaces_wrapper(21, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'ns1.ClassA') && isa(varargin{2},'double')
        varargout{1} = namespaces_wrapper(22, varargin{:});
      else
        error('Arguments do not match any overload of function overloadedGlobalFunction');
      end
end
