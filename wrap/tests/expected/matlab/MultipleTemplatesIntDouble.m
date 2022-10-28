%class MultipleTemplatesIntDouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef MultipleTemplatesIntDouble < handle
  properties
    ptr_MultipleTemplatesIntDouble = 0
  end
  methods
    function obj = MultipleTemplatesIntDouble(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(55, my_ptr);
      else
        error('Arguments do not match any overload of MultipleTemplatesIntDouble constructor');
      end
      obj.ptr_MultipleTemplatesIntDouble = my_ptr;
    end

    function delete(obj)
      class_wrapper(56, obj.ptr_MultipleTemplatesIntDouble);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
