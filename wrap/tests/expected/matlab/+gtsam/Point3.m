%class Point3, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Point3(double x, double y, double z)
%
%-------Methods-------
%norm() : returns double
%
%-------Static Methods-------
%StaticFunctionRet(double z) : returns Point3
%staticFunction() : returns double
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns Point3
%
classdef Point3 < handle
  properties
    ptr_gtsamPoint3 = 0
  end
  methods
    function obj = Point3(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(18, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'double') && isa(varargin{2},'double') && isa(varargin{3},'double')
        my_ptr = geometry_wrapper(19, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gtsam.Point3 constructor');
      end
      obj.ptr_gtsamPoint3 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(20, obj.ptr_gtsamPoint3);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = norm(this, varargin)
      % NORM usage: norm() : returns double
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(21, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function gtsam.Point3.norm');
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(22, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Point3.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = StaticFunctionRet(varargin)
      % STATICFUNCTIONRET usage: StaticFunctionRet(double z) : returns Point3
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(23, varargin{:});
        return
      end

      error('Arguments do not match any overload of function Point3.StaticFunctionRet');
    end

    function varargout = staticFunction(varargin)
      % STATICFUNCTION usage: staticFunction() : returns double
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(24, varargin{:});
        return
      end

      error('Arguments do not match any overload of function Point3.staticFunction');
    end

    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gtsam.Point3
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1
        varargout{1} = geometry_wrapper(25, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Point3.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gtsam.Point3.string_deserialize(sobj);
    end
  end
end
