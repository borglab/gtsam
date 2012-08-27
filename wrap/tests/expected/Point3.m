%-------Constructors-------
%Point3(double x, double y, double z)
%-------Methods-------
%norm()
%-------Static Methods-------
%StaticFunctionRet(double z)
%staticFunction()
%
%For more detailed documentation on GTSAM go to our Doxygen page, which can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
classdef Point3 < handle
  properties
    ptr_Point3 = 0
  end
  methods
    function obj = Point3(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(11, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'double') && isa(varargin{2},'double') && isa(varargin{3},'double')
        my_ptr = geometry_wrapper(12, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of Point3 constructor');
      end
      obj.ptr_Point3 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(13, obj.ptr_Point3);
    end

    function display(obj), obj.print(''); end

    function disp(obj), obj.display; end

    function varargout = norm(this, varargin)
      % norm  norm() : 
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % norm()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(14, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point3.norm');
      end
    end

  end

  methods(Static = true)
    function varargout = StaticFunctionRet(varargin)
      % StaticFunctionRet  StaticFunctionRet(double z) : 
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % StaticFunctionRet(double z)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(15, varargin{:});
      else
        error('Arguments do not match any overload of function Point3.StaticFunctionRet');
      end
    end

    function varargout = StaticFunction(varargin)
      % staticFunction  staticFunction() : 
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % staticFunction()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(16, varargin{:});
      else
        error('Arguments do not match any overload of function Point3.StaticFunction');
      end
    end

  end
end
