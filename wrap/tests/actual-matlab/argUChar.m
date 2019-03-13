function varargout = argUChar(varargin)
      if length(varargin) == 1 && isa(varargin{1},'unsigned char')
        varargout{1} = geometry_wrapper(90, varargin{:});
      else
        error('Arguments do not match any overload of function argUChar');
      end
