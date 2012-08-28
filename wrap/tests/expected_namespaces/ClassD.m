%-------Constructors-------
%CLASSD()
% 
%-------Methods-------
% 
%-------Static Methods-------
%
%For more detailed documentation on GTSAM go to our Doxygen page, which can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
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

    function disp(obj), obj.display; end

  end

  methods(Static = true)
  end
end
