%class ScopedTemplateResult, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%ScopedTemplateResult(Result::Value arg)
%
classdef ScopedTemplateResult < handle
  properties
    ptr_ScopedTemplateResult = 0
  end
  methods
    function obj = ScopedTemplateResult(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        template_wrapper(6, my_ptr);
      elseif nargin == 1 && isa(varargin{1},'Result::Value')
        my_ptr = template_wrapper(7, varargin{1});
      else
        error('Arguments do not match any overload of ScopedTemplateResult constructor');
      end
      obj.ptr_ScopedTemplateResult = my_ptr;
    end

    function delete(obj)
      template_wrapper(8, obj.ptr_ScopedTemplateResult);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
