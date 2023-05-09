function varargout = MultiTemplatedFunctionStringSize_tDouble(varargin)
      if length(varargin) == 2 && isa(varargin{1},'T') && isa(varargin{2},'numeric')
        varargout{1} = functions_wrapper(6, varargin{:});
      else
        error('Arguments do not match any overload of function MultiTemplatedFunctionStringSize_tDouble');
      end
