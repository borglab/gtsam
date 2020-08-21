%class PrimitiveRefdouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%PrimitiveRefdouble()
%
%-------Static Methods-------
%Brutal(double t) : returns PrimitiveRef<double>
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns PrimitiveRefdouble
%
classdef PrimitiveRefdouble < handle
  properties
    ptr_PrimitiveRefdouble = 0
  end
  methods
    function obj = PrimitiveRefdouble(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(79, my_ptr);
      elseif nargin == 0
        my_ptr = geometry_wrapper(80);
      else
        error('Arguments do not match any overload of PrimitiveRefdouble constructor');
      end
      obj.ptr_PrimitiveRefdouble = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(81, obj.ptr_PrimitiveRefdouble);
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
        varargout{1} = geometry_wrapper(82, varargin{:});
        return
      end

      error('Arguments do not match any overload of function PrimitiveRefdouble.Brutal');
    end

  end
end
