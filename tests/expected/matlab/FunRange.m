%class FunRange, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%FunRange()
%
%-------Methods-------
%range(double d) : returns FunRange
%
%-------Static Methods-------
%create() : returns FunRange
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns FunRange
%
classdef FunRange < handle
  properties
    ptr_FunRange = 0
  end
  methods
    function obj = FunRange(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(1);
      else
        error('Arguments do not match any overload of FunRange constructor');
      end
      obj.ptr_FunRange = my_ptr;
    end

    function delete(obj)
      class_wrapper(2, obj.ptr_FunRange);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = range(this, varargin)
      % RANGE usage: range(double d) : returns FunRange
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = class_wrapper(3, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function FunRange.range');
    end

  end

  methods(Static = true)
    function varargout = create(varargin)
      % CREATE usage: create() : returns FunRange
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = class_wrapper(4, varargin{:});
        return
      end

      error('Arguments do not match any overload of function FunRange.create');
    end

  end
end
