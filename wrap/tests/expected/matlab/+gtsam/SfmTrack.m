%class SfmTrack, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Properties-------
%measurements
%
classdef SfmTrack < handle
  properties
    ptr_gtsamSfmTrack = 0
    measurements
  end
  methods
    function obj = SfmTrack(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        special_cases_wrapper(3, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.SfmTrack constructor');
      end
      obj.ptr_gtsamSfmTrack = my_ptr;
    end

    function delete(obj)
      special_cases_wrapper(4, obj.ptr_gtsamSfmTrack);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object

    function varargout = get.measurements(this)
        varargout{1} = special_cases_wrapper(5, this);
        this.measurements = varargout{1};
    end

    function set.measurements(this, value)
        obj.measurements = value;
        special_cases_wrapper(6, this, value);
    end
  end

  methods(Static = true)
  end
end
