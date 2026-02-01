%class Class1, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Class1()
%
classdef Class1 < handle
  properties
    ptr_gtsamClass1 = 0
  end
  methods
    function obj = Class1(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        multiple_files_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = multiple_files_wrapper(1);
      else
        error('Arguments do not match any overload of gtsam.Class1 constructor');
      end
      obj.ptr_gtsamClass1 = my_ptr;
    end

    function delete(obj)
      multiple_files_wrapper(2, obj.ptr_gtsamClass1);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
