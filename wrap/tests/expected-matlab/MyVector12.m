%class MyVector12, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
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
        geometry_wrapper(79, my_ptr);
      elseif nargin == 0
        my_ptr = geometry_wrapper(80);
      else
        error('Arguments do not match any overload of MyVector12 constructor');
      end
      obj.ptr_MyVector12 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(81, obj.ptr_MyVector12);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
