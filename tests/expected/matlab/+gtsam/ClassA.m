%class ClassA, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%ClassA()
%
classdef ClassA < handle
  properties
    ptr_gtsamClassA = 0
  end
  methods
    function obj = ClassA(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        multiple_files_wrapper(6, my_ptr);
      elseif nargin == 0
        my_ptr = multiple_files_wrapper(7);
      else
        error('Arguments do not match any overload of gtsam.ClassA constructor');
      end
      obj.ptr_gtsamClassA = my_ptr;
    end

    function delete(obj)
      multiple_files_wrapper(8, obj.ptr_gtsamClassA);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
