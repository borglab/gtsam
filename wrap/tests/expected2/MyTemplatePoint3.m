%class MyTemplatePoint3, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%MyTemplatePoint3()
%
%-------Methods-------
%templatedMethod(Test t) : returns void
%
classdef MyTemplatePoint3 < MyBase
  properties
    ptr_MyTemplatePoint3 = 0
  end
  methods
    function obj = MyTemplatePoint3(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = geometry_wrapper(49, varargin{2});
        end
        base_ptr = geometry_wrapper(48, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = geometry_wrapper(50);
      else
        error('Arguments do not match any overload of MyTemplatePoint3 constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplatePoint3 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(51, obj.ptr_MyTemplatePoint3);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = templatedMethod(this, varargin)
      % TEMPLATEDMETHOD usage: templatedMethod(Test t) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'Test')
        geometry_wrapper(52, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.templatedMethod');
      end
    end

  end

  methods(Static = true)
  end
end
