function varargout = FindKarcherMeanPose2(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::Pose2')
        varargout{1} = functions_wrapper(30, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanPose2');
      end
end
