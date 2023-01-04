%class Class2, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Class2()
%
classdef Class2 < handle
  properties
    ptr_gtsamClass2 = 0
  end
  methods
    function obj = Class2(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        multiple_files_wrapper(3, my_ptr);
      elseif nargin == 0
        my_ptr = multiple_files_wrapper(4);
      else
        error('Arguments do not match any overload of gtsam.Class2 constructor');
      end
      obj.ptr_gtsamClass2 = my_ptr;
    end

    function delete(obj)
      multiple_files_wrapper(5, obj.ptr_gtsamClass2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
