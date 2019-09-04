%class MyTemplatePoint2, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%MyTemplatePoint2()
%
%-------Methods-------
%accept_T(Point2 value) : returns void
%accept_Tptr(Point2 value) : returns void
%create_MixedPtrs() : returns pair< gtsam::Point2, gtsam::Point2 >
%create_ptrs() : returns pair< gtsam::Point2, gtsam::Point2 >
%return_T(Point2 value) : returns gtsam::Point2
%return_Tptr(Point2 value) : returns gtsam::Point2
%return_ptrs(Point2 p1, Point2 p2) : returns pair< gtsam::Point2, gtsam::Point2 >
%templatedMethodMatrix(Matrix t) : returns Matrix
%templatedMethodPoint2(Point2 t) : returns gtsam::Point2
%templatedMethodPoint3(Point3 t) : returns gtsam::Point3
%templatedMethodVector(Vector t) : returns Vector
%
classdef MyTemplatePoint2 < MyBase
  properties
    ptr_MyTemplatePoint2 = 0
  end
  methods
    function obj = MyTemplatePoint2(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = geometry_wrapper(47, varargin{2});
        end
        base_ptr = geometry_wrapper(46, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = geometry_wrapper(48);
      else
        error('Arguments do not match any overload of MyTemplatePoint2 constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplatePoint2 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(49, obj.ptr_MyTemplatePoint2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(Point2 value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        geometry_wrapper(50, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.accept_T');
      end
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(Point2 value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        geometry_wrapper(51, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.accept_Tptr');
      end
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< gtsam::Point2, gtsam::Point2 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(52, this, varargin{:});
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< gtsam::Point2, gtsam::Point2 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(53, this, varargin{:});
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(Point2 value) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = geometry_wrapper(54, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.return_T');
      end
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(Point2 value) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = geometry_wrapper(55, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.return_Tptr');
      end
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Point2 p1, Point2 p2) : returns pair< gtsam::Point2, gtsam::Point2 >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Point2') && isa(varargin{2},'gtsam.Point2')
        [ varargout{1} varargout{2} ] = geometry_wrapper(56, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.return_ptrs');
      end
    end

    function varargout = templatedMethodMatrix(this, varargin)
      % TEMPLATEDMETHODMATRIX usage: templatedMethodMatrix(Matrix t) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(57, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethod');
      end
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = geometry_wrapper(58, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethod');
      end
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        varargout{1} = geometry_wrapper(59, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethod');
      end
    end

    function varargout = templatedMethodVector(this, varargin)
      % TEMPLATEDMETHODVECTOR usage: templatedMethodVector(Vector t) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = geometry_wrapper(60, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethod');
      end
    end

  end

  methods(Static = true)
  end
end
