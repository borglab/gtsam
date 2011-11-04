classdef Test
  properties
    self = 0
  end
  methods
    function obj = Test(varargin)
      if nargin == 0, obj.self = new_Test_(); end
      if nargin ~= 13 && obj.self == 0, error('Test constructor failed'); end
    end
    function display(obj), obj.print(''); end
    function disp(obj), obj.display; end
  end
end
