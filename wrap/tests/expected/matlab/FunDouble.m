%class FunDouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Methods-------
%multiTemplatedMethodStringSize_t(double d, string t, size_t u) : returns Fun<double>
%templatedMethodString(double d, string t) : returns Fun<double>
%
%-------Static Methods-------
%staticMethodWithThis() : returns Fun<double>
%templatedStaticMethodInt(int m) : returns double
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns FunDouble
%
classdef FunDouble < handle
  properties
    ptr_FunDouble = 0
  end
  methods
    function obj = FunDouble(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(5, my_ptr);
      else
        error('Arguments do not match any overload of FunDouble constructor');
      end
      obj.ptr_FunDouble = my_ptr;
    end

    function delete(obj)
      class_wrapper(6, obj.ptr_FunDouble);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = multiTemplatedMethodStringSize_t(this, varargin)
      % MULTITEMPLATEDMETHODSTRINGSIZE_T usage: multiTemplatedMethodStringSize_t(double d, string t, size_t u) : returns Fun<double>
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 3 && isa(varargin{1},'double') && isa(varargin{2},'char') && isa(varargin{3},'numeric')
        varargout{1} = class_wrapper(7, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function FunDouble.multiTemplatedMethodStringSize_t');
    end

    function varargout = templatedMethodString(this, varargin)
      % TEMPLATEDMETHODSTRING usage: templatedMethodString(double d, string t) : returns Fun<double>
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'char')
        varargout{1} = class_wrapper(8, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function FunDouble.templatedMethodString');
    end

  end

  methods(Static = true)
    function varargout = StaticMethodWithThis(varargin)
      % STATICMETHODWITHTHIS usage: staticMethodWithThis() : returns Fundouble
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = class_wrapper(9, varargin{:});
        return
      end

      error('Arguments do not match any overload of function FunDouble.staticMethodWithThis');
    end

    function varargout = TemplatedStaticMethodInt(varargin)
      % TEMPLATEDSTATICMETHODINT usage: templatedStaticMethodInt(int m) : returns double
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = class_wrapper(10, varargin{:});
        return
      end

      error('Arguments do not match any overload of function FunDouble.templatedStaticMethodInt');
    end

  end
end
