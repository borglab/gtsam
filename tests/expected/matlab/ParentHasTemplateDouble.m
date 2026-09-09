%class ParentHasTemplateDouble, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
classdef ParentHasTemplateDouble < MyTemplate<double>
  properties
    ptr_ParentHasTemplateDouble = 0
  end
  methods
    function obj = ParentHasTemplateDouble(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(55, varargin{2});
        end
        base_ptr = inheritance_wrapper(54, my_ptr);
      else
        error('Arguments do not match any overload of ParentHasTemplateDouble constructor');
      end
      obj = obj@MyTemplatedouble(uint64(5139824614673773682), base_ptr);
      obj.ptr_ParentHasTemplateDouble = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(56, obj.ptr_ParentHasTemplateDouble);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
