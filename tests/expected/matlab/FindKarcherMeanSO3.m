function varargout = FindKarcherMeanSO3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::SO3')
        varargout{1} = functions_wrapper(29, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanSO3');
      end
end
