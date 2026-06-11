function varargout = FindKarcherMeanRot3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::Rot3')
        varargout{1} = functions_wrapper(31, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanRot3');
      end
end
