%class MyVector12, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%MyVector12()
%
classdef MyVector12 < handle
  properties
    ptr_MyVector12 = 0
  end
  methods
    function obj = MyVector12(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(47, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(48);
      else
        error('Arguments do not match any overload of MyVector12 constructor');
      end
      obj.ptr_MyVector12 = my_ptr;
    end

    function delete(obj)
      class_wrapper(49, obj.ptr_MyVector12);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
