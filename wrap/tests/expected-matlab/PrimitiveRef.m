%class PrimitiveRef, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PrimitiveRef()
%
%-------Static Methods-------
%Brutal(double t) : returns This
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns PrimitiveRef
%
classdef PrimitiveRef < handle
  properties
    ptr_PrimitiveRef = 0
  end
  methods
    function obj = PrimitiveRef(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(78, my_ptr);
      elseif nargin == 0
        my_ptr = geometry_wrapper(79);
      else
        error('Arguments do not match any overload of PrimitiveRef constructor');
      end
      obj.ptr_PrimitiveRef = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(80, obj.ptr_PrimitiveRef);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
    function varargout = Brutal(varargin)
      % BRUTAL usage: Brutal(double t) : returns This
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(81, varargin{:});
        return
      end

      error('Arguments do not match any overload of function PrimitiveRef.Brutal');
    end

  end
end
