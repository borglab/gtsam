%class MultipleTemplatesIntFloat, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef MultipleTemplatesIntFloat < handle
  properties
    ptr_MultipleTemplatesIntFloat = 0
  end
  methods
    function obj = MultipleTemplatesIntFloat(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(57, my_ptr);
      else
        error('Arguments do not match any overload of MultipleTemplatesIntFloat constructor');
      end
      obj.ptr_MultipleTemplatesIntFloat = my_ptr;
    end

    function delete(obj)
      class_wrapper(58, obj.ptr_MultipleTemplatesIntFloat);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
