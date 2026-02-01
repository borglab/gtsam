%class ForwardKinematicsFactor, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef ForwardKinematicsFactor < gtsam.BetweenFactor<gtsam.Pose3>
  properties
    ptr_ForwardKinematicsFactor = 0
  end
  methods
    function obj = ForwardKinematicsFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(52, varargin{2});
        end
        base_ptr = inheritance_wrapper(51, my_ptr);
      else
        error('Arguments do not match any overload of ForwardKinematicsFactor constructor');
      end
      obj = obj@gtsam.BetweenFactorPose3(uint64(5139824614673773682), base_ptr);
      obj.ptr_ForwardKinematicsFactor = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(53, obj.ptr_ForwardKinematicsFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
