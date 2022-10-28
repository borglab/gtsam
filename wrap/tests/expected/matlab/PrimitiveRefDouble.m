%class PrimitiveRefDouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%PrimitiveRefDouble()
%
%-------Static Methods-------
%Brutal(double t) : returns PrimitiveRef<double>
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns PrimitiveRefDouble
%
classdef PrimitiveRefDouble < handle
  properties
    ptr_PrimitiveRefDouble = 0
  end
  methods
    function obj = PrimitiveRefDouble(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(45, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(46);
      else
        error('Arguments do not match any overload of PrimitiveRefDouble constructor');
      end
      obj.ptr_PrimitiveRefDouble = my_ptr;
    end

    function delete(obj)
      class_wrapper(47, obj.ptr_PrimitiveRefDouble);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
    function varargout = Brutal(varargin)
      % BRUTAL usage: Brutal(double t) : returns PrimitiveRefdouble
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = class_wrapper(48, varargin{:});
        return
      end

      error('Arguments do not match any overload of function PrimitiveRefDouble.Brutal');
    end

  end
end
