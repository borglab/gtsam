%class Point3, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Point3(double x, double y, double z)
%
%-------Methods-------
%norm() : returns double
%
%-------Static Methods-------
%StaticFunctionRet(double z) : returns gtsam::Point3
%staticFunction() : returns double
%
classdef Point3 < handle
  properties
    ptr_gtsamPoint3 = 0
  end
  methods
    function obj = Point3(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(12, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'double') && isa(varargin{2},'double') && isa(varargin{3},'double')
        my_ptr = geometry_wrapper(13, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gtsam.Point3 constructor');
      end
      obj.ptr_gtsamPoint3 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(14, obj.ptr_gtsamPoint3);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = norm(this, varargin)
      % NORM usage: norm() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = geometry_wrapper(15, this, varargin{:});
    end

  end

  methods(Static = true)
    function varargout = StaticFunctionRet(varargin)
      % STATICFUNCTIONRET usage: StaticFunctionRet(double z) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = geometry_wrapper(16, varargin{:});
    end

    function varargout = StaticFunction(varargin)
      % STATICFUNCTION usage: staticFunction() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = geometry_wrapper(17, varargin{:});
    end

  end
end
