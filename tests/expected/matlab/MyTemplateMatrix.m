%class MyTemplateMatrix, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%MyTemplateMatrix()
%
%-------Methods-------
%accept_T(Matrix value) : returns void
%accept_Tptr(Matrix value) : returns void
%create_MixedPtrs() : returns pair< Matrix, Matrix >
%create_ptrs() : returns pair< Matrix, Matrix >
%return_T(Matrix value) : returns Matrix
%return_Tptr(Matrix value) : returns Matrix
%return_ptrs(Matrix p1, Matrix p2) : returns pair< Matrix, Matrix >
%templatedMethodMatrix(Matrix t) : returns Matrix
%templatedMethodPoint2(Point2 t) : returns Point2
%templatedMethodPoint3(Point3 t) : returns Point3
%templatedMethodVector(Vector t) : returns Vector
%
%-------Static Methods-------
%Level(Matrix K) : returns MyTemplate<Matrix>
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns MyTemplateMatrix
%
classdef MyTemplateMatrix < MyBase
  properties
    ptr_MyTemplateMatrix = 0
  end
  methods
    function obj = MyTemplateMatrix(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = inheritance_wrapper(20, varargin{2});
        end
        base_ptr = inheritance_wrapper(19, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = inheritance_wrapper(21);
      else
        error('Arguments do not match any overload of MyTemplateMatrix constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplateMatrix = my_ptr;
    end

    function delete(obj)
      inheritance_wrapper(22, obj.ptr_MyTemplateMatrix);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(Matrix value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        inheritance_wrapper(23, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.accept_T');
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(Matrix value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        inheritance_wrapper(24, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.accept_Tptr');
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< Matrix, Matrix >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(25, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.create_MixedPtrs');
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< Matrix, Matrix >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = inheritance_wrapper(26, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.create_ptrs');
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(Matrix value) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(27, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.return_T');
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(Matrix value) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(28, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.return_Tptr');
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Matrix p1, Matrix p2) : returns pair< Matrix, Matrix >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        [ varargout{1} varargout{2} ] = inheritance_wrapper(29, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.return_ptrs');
    end

    function varargout = templatedMethodMatrix(this, varargin)
      % TEMPLATEDMETHODMATRIX usage: templatedMethodMatrix(Matrix t) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(30, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethodMatrix');
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==2 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(31, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethodPoint2');
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns Point3
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},1)==3 && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(32, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethodPoint3');
    end

    function varargout = templatedMethodVector(this, varargin)
      % TEMPLATEDMETHODVECTOR usage: templatedMethodVector(Vector t) : returns Vector
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = inheritance_wrapper(33, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethodVector');
    end

  end

  methods(Static = true)
    function varargout = Level(varargin)
      % LEVEL usage: Level(Matrix K) : returns MyTemplateMatrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = inheritance_wrapper(34, varargin{:});
        return
      end

      error('Arguments do not match any overload of function MyTemplateMatrix.Level');
    end

  end
end
