%class MyFactorPosePoint2, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%MyFactorPosePoint2(size_t key1, size_t key2, double measured, Base noiseModel)
%
classdef MyFactorPosePoint2 < handle
  properties
    ptr_MyFactorPosePoint2 = 0
  end
  methods
    function obj = MyFactorPosePoint2(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(74, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'gtsam.noiseModel.Base')
        my_ptr = geometry_wrapper(75, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of MyFactorPosePoint2 constructor');
      end
      obj.ptr_MyFactorPosePoint2 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(76, obj.ptr_MyFactorPosePoint2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
