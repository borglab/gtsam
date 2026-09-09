function varargout = FindKarcherMeanSO4(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::SO4')
        varargout{1} = functions_wrapper(30, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanSO4');
      end
end
