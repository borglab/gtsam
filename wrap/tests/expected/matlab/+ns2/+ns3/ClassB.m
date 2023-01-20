%class ClassB, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%ClassB()
%
classdef ClassB < handle
  properties
    ptr_ns2ns3ClassB = 0
  end
  methods
    function obj = ClassB(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        namespaces_wrapper(14, my_ptr);
      elseif nargin == 0
        my_ptr = namespaces_wrapper(15);
      else
        error('Arguments do not match any overload of ns2.ns3.ClassB constructor');
      end
      obj.ptr_ns2ns3ClassB = my_ptr;
    end

    function delete(obj)
      namespaces_wrapper(16, obj.ptr_ns2ns3ClassB);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
