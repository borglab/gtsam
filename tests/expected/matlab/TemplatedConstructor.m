%class TemplatedConstructor, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%TemplatedConstructor()
%TemplatedConstructor(string arg)
%TemplatedConstructor(int arg)
%TemplatedConstructor(double arg)
%
classdef TemplatedConstructor < handle
  properties
    ptr_TemplatedConstructor = 0
  end
  methods
    function obj = TemplatedConstructor(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        template_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = template_wrapper(1);
      elseif nargin == 1 && isa(varargin{1},'char')
        my_ptr = template_wrapper(2, varargin{1});
      elseif nargin == 1 && isa(varargin{1},'numeric')
        my_ptr = template_wrapper(3, varargin{1});
      elseif nargin == 1 && isa(varargin{1},'double')
        my_ptr = template_wrapper(4, varargin{1});
      else
        error('Arguments do not match any overload of TemplatedConstructor constructor');
      end
      obj.ptr_TemplatedConstructor = my_ptr;
    end

    function delete(obj)
      template_wrapper(5, obj.ptr_TemplatedConstructor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
