%class GeneralSFMFactorCal3Bundler, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef GeneralSFMFactorCal3Bundler < handle
  properties
    ptr_gtsamGeneralSFMFactorCal3Bundler = 0
  end
  methods
    function obj = GeneralSFMFactorCal3Bundler(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        special_cases_wrapper(9, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.GeneralSFMFactorCal3Bundler constructor');
      end
      obj.ptr_gtsamGeneralSFMFactorCal3Bundler = my_ptr;
    end

    function delete(obj)
      special_cases_wrapper(10, obj.ptr_gtsamGeneralSFMFactorCal3Bundler);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
