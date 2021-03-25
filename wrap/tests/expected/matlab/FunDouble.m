%class FunDouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Methods-------
%dhamaalString(double d, string t) : returns Fun<double>
%
%-------Static Methods-------
%divertido() : returns Fun<double>
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
    function varargout = dhamaalString(this, varargin)
      % DHAMAALSTRING usage: dhamaalString(double d, string t) : returns Fun<double>
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'char')
        varargout{1} = class_wrapper(7, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function FunDouble.dhamaalString');
    end

  end

  methods(Static = true)
    function varargout = Divertido(varargin)
      % DIVERTIDO usage: divertido() : returns Fundouble
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = class_wrapper(8, varargin{:});
        return
      end

      error('Arguments do not match any overload of function FunDouble.divertido');
    end

  end
end
