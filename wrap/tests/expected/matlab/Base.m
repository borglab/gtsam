%class Base, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Static Methods-------
%Create(double x) : returns gtsam::Base
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns Base
%
classdef Base < handle
  properties
    ptr_Base = 0
  end
  methods
    function obj = Base(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(58, varargin{2});
        end
        inheritance_wrapper(57, my_ptr);
      else
        error('Arguments do not match any overload of Base constructor');
      end
      obj.ptr_Base = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(59, obj.ptr_Base);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
    function varargout = Create(varargin)
      % CREATE usage: Create(double x) : returns gtsam.Base
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(60, varargin{:});
        return
      end

      error('Arguments do not match any overload of function Base.Create');
    end

  end
end
