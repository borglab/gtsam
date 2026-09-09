function varargout = triangulatePoint3Cal3_S2(varargin)
      if length(varargin) == 6 && isa(varargin{1},'gtsam.Pose3Vector') && isa(varargin{2},'gtsam.Cal3_S2') && isa(varargin{3},'gtsam.Point2Vector') && isa(varargin{4},'double') && isa(varargin{5},'logical') && isa(varargin{6},'gtsam.SharedNoiseModel')
        varargout{1} = functions_wrapper(26, varargin{:});
      elseif length(varargin) == 5 && isa(varargin{1},'gtsam.Pose3Vector') && isa(varargin{2},'gtsam.Cal3_S2') && isa(varargin{3},'gtsam.Point2Vector') && isa(varargin{4},'double') && isa(varargin{5},'logical')
        varargout{1} = functions_wrapper(27, varargin{:});
      else
        error('Arguments do not match any overload of function triangulatePoint3Cal3_S2');
      end
end
