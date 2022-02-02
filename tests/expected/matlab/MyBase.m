%class MyBase, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef MyBase < handle
  properties
    ptr_MyBase = 0
  end
  methods
    function obj = MyBase(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(1, varargin{2});
        end
        inheritance_wrapper(0, my_ptr);
      else
        error('Arguments do not match any overload of MyBase constructor');
      end
      obj.ptr_MyBase = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(2, obj.ptr_MyBase);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
