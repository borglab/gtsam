function varargout = EliminateDiscrete(varargin)
      if length(varargin) == 2 && isa(varargin{1},'gtsam.DiscreteFactorGraph') && isa(varargin{2},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = functions_wrapper(25, varargin{:});
      else
        error('Arguments do not match any overload of function EliminateDiscrete');
      end
end
