function varargout = argChar(varargin)
      if length(varargin) == 1 && isa(varargin{1},'char')
        varargout{1} = geometry_wrapper(89, varargin{:});
      else
        error('Arguments do not match any overload of function argChar');
      end
