function varargout = FindKarcherMeanPose3(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::Pose3')
        varargout{1} = functions_wrapper(31, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanPose3');
      end
end
