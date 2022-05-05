%class SfmTrack, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef SfmTrack < handle
  properties
    ptr_gtsamSfmTrack = 0
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
  end

  methods(Static = true)
  end
end
