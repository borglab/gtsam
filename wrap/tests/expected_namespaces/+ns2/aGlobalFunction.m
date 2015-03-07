function varargout = aGlobalFunction(varargin)
if length(varargin) == 0
    varargout{1} = testNamespaces_wrapper(23, varargin{:});
else
    error('Arguments do not match any overload of function ns2.aGlobalFunction');
end
