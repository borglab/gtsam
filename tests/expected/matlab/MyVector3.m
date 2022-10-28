%class MyVector3, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%MyVector3()
%
classdef MyVector3 < handle
  properties
    ptr_MyVector3 = 0
  end
  methods
    function obj = MyVector3(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(49, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(50);
      else
        error('Arguments do not match any overload of MyVector3 constructor');
      end
      obj.ptr_MyVector3 = my_ptr;
    end

    function delete(obj)
      class_wrapper(51, obj.ptr_MyVector3);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
