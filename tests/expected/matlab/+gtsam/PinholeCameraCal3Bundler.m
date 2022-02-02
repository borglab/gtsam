%class PinholeCameraCal3Bundler, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef PinholeCameraCal3Bundler < handle
  properties
    ptr_gtsamPinholeCameraCal3Bundler = 0
  end
  methods
    function obj = PinholeCameraCal3Bundler(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        special_cases_wrapper(5, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.PinholeCameraCal3Bundler constructor');
      end
      obj.ptr_gtsamPinholeCameraCal3Bundler = my_ptr;
    end

    function delete(obj)
      special_cases_wrapper(6, obj.ptr_gtsamPinholeCameraCal3Bundler);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
