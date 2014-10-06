function varargout = overloadedGlobalFunction(varargin)
      if length(varargin) == 1 && isa(varargin{1},'ns1.ClassA')
        varargout{1} = testNamespaces_wrapper(24, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'ns1.ClassA') && isa(varargin{2},'double')
        varargout{1} = testNamespaces_wrapper(25, varargin{:});
      else
        error('Arguments do not match any overload of function ns2.overloadedGlobalFunction');
      end
