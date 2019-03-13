function varargout = create_MixedPtrs(varargin)
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(112, varargin{:});
      else
        error('Arguments do not match any overload of function create_MixedPtrs');
      end
