%class MyTemplatePoint2, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%MyTemplatePoint2()
%
%-------Methods-------
%accept_T(Point2 value) : returns void
%accept_Tptr(Point2 value) : returns void
%create_MixedPtrs() : returns pair< Point2, Point2 >
%create_ptrs() : returns pair< Point2, Point2 >
%return_T(Point2 value) : returns Point2
%return_Tptr(Point2 value) : returns Point2
%return_ptrs(Point2 p1, Point2 p2) : returns pair< Point2, Point2 >
%templatedMethodMatrix(Matrix t) : returns Matrix
%templatedMethodPoint2(Point2 t) : returns Point2
%templatedMethodPoint3(Point3 t) : returns Point3
%templatedMethodVector(Vector t) : returns Vector
%
%-------Static Methods-------
%Level(Point2 K) : returns MyTemplate<Point2>
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns MyTemplatePoint2
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
          my_ptr = inheritance_wrapper(4, varargin{2});
        end
        base_ptr = inheritance_wrapper(3, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = inheritance_wrapper(5);
      else
        error('Arguments do not match any overload of MyTemplatePoint2 constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplatePoint2 = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(6, obj.ptr_MyTemplatePoint2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(Point2 value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        inheritance_wrapper(7, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.accept_T');
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(Point2 value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        inheritance_wrapper(8, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.accept_Tptr');
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< Point2, Point2 >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(9, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.create_MixedPtrs');
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< Point2, Point2 >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(10, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.create_ptrs');
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(Point2 value) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(11, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.return_T');
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(Point2 value) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(12, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.return_Tptr');
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Point2 p1, Point2 p2) : returns pair< Point2, Point2 >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},1)==2 && size(varargin{2},2)==1
        [ varargout{1} varargout{2} ] = inheritance_wrapper(13, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.return_ptrs');
    end

    function varargout = templatedMethodMatrix(this, varargin)
      % TEMPLATEDMETHODMATRIX usage: templatedMethodMatrix(Matrix t) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(14, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethodMatrix');
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(15, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethodPoint2');
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns Point3
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==3 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(16, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethodPoint3');
    end

    function varargout = templatedMethodVector(this, varargin)
      % TEMPLATEDMETHODVECTOR usage: templatedMethodVector(Vector t) : returns Vector
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(17, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplatePoint2.templatedMethodVector');
    end

  end

  methods(Static = true)
    function varargout = Level(varargin)
      % LEVEL usage: Level(Point2 K) : returns MyTemplatePoint2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(18, varargin{:});
        return
      end

      error('Arguments do not match any overload of function MyTemplatePoint2.Level');
    end

  end
end
