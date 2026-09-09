function varargout = FindKarcherMeanRot2(varargin)
      if length(varargin) == 1 && isa(varargin{1},'std.vectorgtsam::Rot2')
        varargout{1} = functions_wrapper(29, varargin{:});
      else
        error('Arguments do not match any overload of function FindKarcherMeanRot2');
      end
end
