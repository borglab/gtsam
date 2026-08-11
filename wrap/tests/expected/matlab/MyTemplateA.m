%class MyTemplateA, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%MyTemplateA()
%
%-------Methods-------
%accept_T(A value) : returns void
%accept_Tptr(A value) : returns void
%create_MixedPtrs() : returns pair< A, A >
%create_ptrs() : returns pair< A, A >
%return_T(A value) : returns A
%return_Tptr(A value) : returns A
%return_ptrs(A p1, A p2) : returns pair< A, A >
%templatedMethodMatrix(Matrix t) : returns Matrix
%templatedMethodPoint2(Point2 t) : returns Point2
%templatedMethodPoint3(Point3 t) : returns Point3
%templatedMethodVector(Vector t) : returns Vector
%
%-------Static Methods-------
%Level(A K) : returns MyTemplate<A>
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns MyTemplateA
%
classdef MyTemplateA < MyBase
  properties
    ptr_MyTemplateA = 0
  end
  methods
    function obj = MyTemplateA(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(36, varargin{2});
        end
        base_ptr = inheritance_wrapper(35, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = inheritance_wrapper(37);
      else
        error('Arguments do not match any overload of MyTemplateA constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplateA = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(38, obj.ptr_MyTemplateA);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(A value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'A')
        inheritance_wrapper(39, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.accept_T');
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(A value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'A')
        inheritance_wrapper(40, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.accept_Tptr');
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< A, A >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(41, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.create_MixedPtrs');
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< A, A >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(42, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.create_ptrs');
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(A value) : returns A
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'A')
        varargout{1} = inheritance_wrapper(43, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.return_T');
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(A value) : returns A
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'A')
        varargout{1} = inheritance_wrapper(44, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.return_Tptr');
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(A p1, A p2) : returns pair< A, A >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'A') && isa(varargin{2},'A')
        [ varargout{1} varargout{2} ] = inheritance_wrapper(45, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.return_ptrs');
    end

    function varargout = templatedMethodMatrix(this, varargin)
      % TEMPLATEDMETHODMATRIX usage: templatedMethodMatrix(Matrix t) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(46, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.templatedMethodMatrix');
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(47, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.templatedMethodPoint2');
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns Point3
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==3 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(48, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.templatedMethodPoint3');
    end

    function varargout = templatedMethodVector(this, varargin)
      % TEMPLATEDMETHODVECTOR usage: templatedMethodVector(Vector t) : returns Vector
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(49, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateA.templatedMethodVector');
    end

  end

  methods(Static = true)
    function varargout = Level(varargin)
      % LEVEL usage: Level(A K) : returns MyTemplateA
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'A')
        varargout{1} = inheritance_wrapper(50, varargin{:});
        return
      end

      error('Arguments do not match any overload of function MyTemplateA.Level');
    end

  end
end
