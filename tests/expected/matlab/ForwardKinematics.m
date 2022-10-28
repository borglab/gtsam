%class ForwardKinematics, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%ForwardKinematics(Robot robot, string start_link_name, string end_link_name, Values joint_angles, Pose3 l2Tp)
%
classdef ForwardKinematics < handle
  properties
    ptr_ForwardKinematics = 0
  end
  methods
    function obj = ForwardKinematics(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(59, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'gtdynamics.Robot') && isa(varargin{2},'char') && isa(varargin{3},'char') && isa(varargin{4},'gtsam.Values') && isa(varargin{5},'gtsam.Pose3')
        my_ptr = class_wrapper(60, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      elseif nargin == 4 && isa(varargin{1},'gtdynamics.Robot') && isa(varargin{2},'char') && isa(varargin{3},'char') && isa(varargin{4},'gtsam.Values')
        my_ptr = class_wrapper(61, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of ForwardKinematics constructor');
      end
      obj.ptr_ForwardKinematics = my_ptr;
    end

    function delete(obj)
      class_wrapper(62, obj.ptr_ForwardKinematics);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
