classdef Test
  properties
    self = 0
  end
  methods
    function obj = Test(varargin)
      if (nargin == 0), obj.self = new_Test_(); end
      if (nargin == 2 & isa(varargin{1},'double') & isa(varargin{2},'double')), obj.self = new_Test_dM(varargin{1},varargin{2}); end
      if nargin ~= 13 && obj.self == 0, error('Test constructor failed'); end
    end
    function display(obj), obj.print(''); end
    function disp(obj), obj.display; end
  end
end
