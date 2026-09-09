%class FastSet, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%FastSet()
%
classdef FastSet < handle
  properties
    ptr_FastSet = 0
  end
  methods
    function obj = FastSet(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(73, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(74);
      else
        error('Arguments do not match any overload of FastSet constructor');
      end
      obj.ptr_FastSet = my_ptr;
    end

    function delete(obj)
      class_wrapper(75, obj.ptr_FastSet);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
