function varargout = FindKarcherMeanPoint3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::Point3')
        varargout{1} = functions_wrapper(28, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanPoint3');
      end
end
