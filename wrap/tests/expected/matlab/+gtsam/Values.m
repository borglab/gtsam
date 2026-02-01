%class Values, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Values()
%Values(Values other)
%
%-------Methods-------
%insert(size_t j, Vector vector) : returns void
%insert(size_t j, Matrix matrix) : returns void
%
classdef Values < handle
  properties
    ptr_gtsamValues = 0
  end
  methods
    function obj = Values(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        namespaces_wrapper(26, my_ptr);
      elseif nargin == 0
        my_ptr = namespaces_wrapper(27);
      elseif nargin == 1 && isa(varargin{1},'gtsam.Values')
        my_ptr = namespaces_wrapper(28, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.Values constructor');
      end
      obj.ptr_gtsamValues = my_ptr;
    end

    function delete(obj)
      namespaces_wrapper(29, obj.ptr_gtsamValues);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = insert(this, varargin)
      % INSERT usage: insert(size_t j, Vector vector) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && size(varargin{2},2)==1
        namespaces_wrapper(30, this, varargin{:});
        return
      end
      % INSERT usage: insert(size_t j, Matrix matrix) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        namespaces_wrapper(31, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function gtsam.Values.insert');
    end

  end

  methods(Static = true)
  end
end
