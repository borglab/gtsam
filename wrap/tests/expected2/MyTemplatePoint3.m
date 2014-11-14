%class MyTemplatePoint3, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%MyTemplatePoint3()
%
%-------Methods-------
%accept_T(Point3 value) : returns void
%accept_Tptr(Point3 value) : returns void
%create_MixedPtrs() : returns pair< gtsam::Point3, gtsam::Point3 >
%create_ptrs() : returns pair< gtsam::Point3, gtsam::Point3 >
%return_T(Point3 value) : returns gtsam::Point3
%return_Tptr(Point3 value) : returns gtsam::Point3
%return_ptrs(Point3 p1, Point3 p2) : returns pair< gtsam::Point3, gtsam::Point3 >
%templatedMethodPoint2(Point2 t) : returns void
%templatedMethodPoint3(Point3 t) : returns void
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
          my_ptr = geometry_wrapper(57, varargin{2});
        end
        base_ptr = geometry_wrapper(56, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = geometry_wrapper(58);
      else
        error('Arguments do not match any overload of MyTemplatePoint3 constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplatePoint3 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(59, obj.ptr_MyTemplatePoint3);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(Point3 value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        geometry_wrapper(60, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.accept_T');
      end
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(Point3 value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        geometry_wrapper(61, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.accept_Tptr');
      end
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< gtsam::Point3, gtsam::Point3 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(62, this, varargin{:});
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< gtsam::Point3, gtsam::Point3 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(63, this, varargin{:});
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(Point3 value) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        varargout{1} = geometry_wrapper(64, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.return_T');
      end
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(Point3 value) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        varargout{1} = geometry_wrapper(65, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.return_Tptr');
      end
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Point3 p1, Point3 p2) : returns pair< gtsam::Point3, gtsam::Point3 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Point3') && isa(varargin{2},'gtsam.Point3')
        [ varargout{1} varargout{2} ] = geometry_wrapper(66, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.return_ptrs');
      end
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        geometry_wrapper(67, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.templatedMethod');
      end
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        geometry_wrapper(68, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint3.templatedMethod');
      end
    end

  end

  methods(Static = true)
  end
end
