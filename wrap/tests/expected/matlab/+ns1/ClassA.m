%class ClassA, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%ClassA()
%
classdef ClassA < handle
  properties
    ptr_ns1ClassA = 0
  end
  methods
    function obj = ClassA(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        namespaces_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = namespaces_wrapper(1);
      else
        error('Arguments do not match any overload of ns1.ClassA constructor');
      end
      obj.ptr_ns1ClassA = my_ptr;
    end

    function delete(obj)
      namespaces_wrapper(2, obj.ptr_ns1ClassA);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
