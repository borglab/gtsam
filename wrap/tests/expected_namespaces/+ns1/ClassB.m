%-------Constructors-------
%ClassB()
%-------Methods-------
%-------Static Methods-------
%
%For more detailed documentation on GTSAM go to our Doxygen page, which can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
classdef ClassB < handle
  properties
    ptr_ns1ClassB = 0
  end
  methods
    function obj = ClassB(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        testNamespaces_wrapper(3, my_ptr);
      elseif nargin == 0
        my_ptr = testNamespaces_wrapper(4);
      else
        error('Arguments do not match any overload of ns1.ClassB constructor');
      end
      obj.ptr_ns1ClassB = my_ptr;
    end

    function delete(obj)
      testNamespaces_wrapper(5, obj.ptr_ns1ClassB);
    end

    function display(obj), obj.print(''); end

    function disp(obj), obj.display; end

  end

  methods(Static = true)
  end
end
