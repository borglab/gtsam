%class SmartProjectionRigFactorPinholeCameraCal3_S2, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Methods-------
%add(PinholeCamera<gtsam::Cal3_S2>::Measurement measured, Key poseKey, size_t cameraId) : returns void
%
classdef SmartProjectionRigFactorPinholeCameraCal3_S2 < gtsam.SmartProjectionFactor<gtsam.PinholeCamera<gtsam.Cal3_S2>>
  properties
    ptr_SmartProjectionRigFactorPinholeCameraCal3_S2 = 0
  end
  methods
    function obj = SmartProjectionRigFactorPinholeCameraCal3_S2(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        base_ptr = class_wrapper(83, my_ptr);
      else
        error('Arguments do not match any overload of SmartProjectionRigFactorPinholeCameraCal3_S2 constructor');
      end
      obj = obj@gtsam.SmartProjectionFactorgtsam::PinholeCamera<gtsam::Cal3_S2>(uint64(5139824614673773682), base_ptr);
      obj.ptr_SmartProjectionRigFactorPinholeCameraCal3_S2 = my_ptr;
    end

    function delete(obj)
      class_wrapper(84, obj.ptr_SmartProjectionRigFactorPinholeCameraCal3_S2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = add(this, varargin)
      % ADD usage: add(PinholeCamera<gtsam::Cal3_S2>::Measurement measured, Key poseKey, size_t cameraId) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 3 && isa(varargin{1},'gtsam.PinholeCamera<gtsam::Cal3_S2>::Measurement') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric')
        class_wrapper(85, this, varargin{:});
        return
      end
      % ADD usage: add(PinholeCamera<gtsam::Cal3_S2>::Measurement measured, Key poseKey) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'gtsam.PinholeCamera<gtsam::Cal3_S2>::Measurement') && isa(varargin{2},'numeric')
        class_wrapper(86, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function SmartProjectionRigFactorPinholeCameraCal3_S2.add');
    end

  end

  methods(Static = true)
  end
end
