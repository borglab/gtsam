%class ClassD, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ClassD()
%
classdef ClassD < handle
  properties
    ptr_ClassD = 0
  end
  methods
    function obj = ClassD(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        testNamespaces_wrapper(19, my_ptr);
      elseif nargin == 0
        my_ptr = testNamespaces_wrapper(20);
      else
        error('Arguments do not match any overload of ClassD constructor');
      end
      obj.ptr_ClassD = my_ptr;
    end

    function delete(obj)
      testNamespaces_wrapper(21, obj.ptr_ClassD);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
