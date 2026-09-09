%class Pet, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Pet(string name, Kind type)
%
%-------Properties-------
%name
%type
%
%-------Methods-------
%getColor() : returns Color
%setColor(Color color) : returns void
%
classdef Pet < handle
  properties
    ptr_Pet = 0
    name
    type
  end
  methods
    function obj = Pet(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        enum_wrapper(0, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'char') && isa(varargin{2},'Pet.Kind')
        my_ptr = enum_wrapper(1, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of Pet constructor');
      end
      obj.ptr_Pet = my_ptr;
    end

    function delete(obj)
      enum_wrapper(2, obj.ptr_Pet);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getColor(this, varargin)
      % GETCOLOR usage: getColor() : returns Color
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = enum_wrapper(3, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Pet.getColor');
    end

    function varargout = setColor(this, varargin)
      % SETCOLOR usage: setColor(Color color) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'Color')
        enum_wrapper(4, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Pet.setColor');
    end


    function varargout = get.name(this)
        varargout{1} = enum_wrapper(5, this);
        this.name = varargout{1};
    end

    function set.name(this, value)
        obj.name = value;
        enum_wrapper(6, this, value);
    end

    function varargout = get.type(this)
        varargout{1} = enum_wrapper(7, this);
        this.type = varargout{1};
    end

    function set.type(this, value)
        obj.type = value;
        enum_wrapper(8, this, value);
    end
  end

  methods(Static = true)
  end
end
