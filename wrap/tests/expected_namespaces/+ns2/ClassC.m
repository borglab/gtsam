%class ClassC, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ClassC()
%
classdef ClassC < handle
  properties
    ptr_ns2ClassC = 0
  end
  methods
    function obj = ClassC(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        testNamespaces_wrapper(16, my_ptr);
      elseif nargin == 0
        my_ptr = testNamespaces_wrapper(17);
      else
        error('Arguments do not match any overload of ns2.ClassC constructor');
      end
      obj.ptr_ns2ClassC = my_ptr;
    end

    function delete(obj)
      testNamespaces_wrapper(18, obj.ptr_ns2ClassC);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
