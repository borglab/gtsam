function varargout = MultiTemplatedFunctionDoubleSize_tDouble(varargin)
      if length(varargin) == 2 && isa(varargin{1},'T') && isa(varargin{2},'numeric')
        varargout{1} = functions_wrapper(7, varargin{:});
      else
        error('Arguments do not match any overload of function MultiTemplatedFunctionDoubleSize_tDouble');
      end
end
