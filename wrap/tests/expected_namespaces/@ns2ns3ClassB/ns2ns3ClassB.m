classdef ns2ns3ClassB
  properties
    self = 0
  end
  methods
    function obj = ns2ns3ClassB(varargin)
      if nargin == 0, obj.self = new_ns2ns3ClassB_(); end
      if nargin ~= 13 && obj.self == 0, error('ns2ns3ClassB constructor failed'); end
    end
    function display(obj), obj.print(''); end
    function disp(obj), obj.display; end
  end
end
