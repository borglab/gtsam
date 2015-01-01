%class MyBase, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
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
          my_ptr = geometry_wrapper(42, varargin{2});
        end
        geometry_wrapper(41, my_ptr);
      else
        error('Arguments do not match any overload of MyBase constructor');
      end
      obj.ptr_MyBase = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(43, obj.ptr_MyBase);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
