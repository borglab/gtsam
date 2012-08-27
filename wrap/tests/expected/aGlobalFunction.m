function varargout = aGlobalFunction(varargin)
if length(varargin) == 0
    varargout{1} = geometry_wrapper(40, varargin{:});
else
    error('Arguments do not match any overload of function aGlobalFunction');
end
