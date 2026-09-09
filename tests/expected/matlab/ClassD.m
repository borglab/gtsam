%class ClassD, see Doxygen page for details
%at https://gtsam.org/doxygen/
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
        namespaces_wrapper(23, my_ptr);
      elseif nargin == 0
        my_ptr = namespaces_wrapper(24);
      else
        error('Arguments do not match any overload of ClassD constructor');
      end
      obj.ptr_ClassD = my_ptr;
    end

    function delete(obj)
      namespaces_wrapper(25, obj.ptr_ClassD);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
