function varargout = aGlobalFunction(varargin)
if length(varargin) == 0
    varargout{1} = testNamespaces_wrapper(22, varargin{:});
else
    error('Arguments do not match any overload of function ns1.aGlobalFunction');
end
