function varargout = load2D(varargin)
      if length(varargin) == 5 && isa(varargin{1},'char') && isa(varargin{2},'Test') && isa(varargin{3},'numeric') && isa(varargin{4},'logical') && isa(varargin{5},'logical')
        [ varargout{1} varargout{2} ] = functions_wrapper(0, varargin{:});
      elseif length(varargin) == 5 && isa(varargin{1},'char') && isa(varargin{2},'gtsam.noiseModel.Diagonal') && isa(varargin{3},'numeric') && isa(varargin{4},'logical') && isa(varargin{5},'logical')
        [ varargout{1} varargout{2} ] = functions_wrapper(1, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'char') && isa(varargin{2},'gtsam.noiseModel.Diagonal')
        [ varargout{1} varargout{2} ] = functions_wrapper(2, varargin{:});
      else
        error('Arguments do not match any overload of function load2D');
      end
end
