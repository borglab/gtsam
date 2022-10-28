%class Test, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%Test()
%Test(double a, Matrix b)
%
%-------Properties-------
%model_ptr
%value
%name
%
%-------Methods-------
%arg_EigenConstRef(Matrix value) : returns void
%create_MixedPtrs() : returns pair< Test, Test >
%create_ptrs() : returns pair< Test, Test >
%get_container() : returns std::vector<testing::Test>
%lambda() : returns void
%markdown(KeyFormatter keyFormatter) : returns string
%print() : returns void
%return_Point2Ptr(bool value) : returns Point2
%return_Test(Test value) : returns Test
%return_TestPtr(Test value) : returns Test
%return_bool(bool value) : returns bool
%return_double(double value) : returns double
%return_field(Test t) : returns bool
%return_int(int value) : returns int
%return_matrix1(Matrix value) : returns Matrix
%return_matrix2(Matrix value) : returns Matrix
%return_pair(Vector v, Matrix A) : returns pair< Vector, Matrix >
%return_pair(Vector v) : returns pair< Vector, Matrix >
%return_ptrs(Test p1, Test p2) : returns pair< Test, Test >
%return_size_t(size_t value) : returns size_t
%return_string(string value) : returns string
%return_vector1(Vector value) : returns Vector
%return_vector2(Vector value) : returns Vector
%set_container(vector<Test> container) : returns void
%set_container(vector<Test> container) : returns void
%set_container(vector<Test> container) : returns void
%
classdef Test < handle
  properties
    ptr_Test = 0
    model_ptr
    value
    name
  end
  methods
    function obj = Test(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        class_wrapper(12, my_ptr);
      elseif nargin == 0
        my_ptr = class_wrapper(13);
      elseif nargin == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        my_ptr = class_wrapper(14, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of Test constructor');
      end
      obj.ptr_Test = my_ptr;
    end

    function delete(obj)
      class_wrapper(15, obj.ptr_Test);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = arg_EigenConstRef(this, varargin)
      % ARG_EIGENCONSTREF usage: arg_EigenConstRef(Matrix value) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        class_wrapper(16, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.arg_EigenConstRef');
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< Test, Test >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = class_wrapper(17, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.create_MixedPtrs');
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< Test, Test >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = class_wrapper(18, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.create_ptrs');
    end

    function varargout = get_container(this, varargin)
      % GET_CONTAINER usage: get_container() : returns std.vectorTest
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = class_wrapper(19, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.get_container');
    end

    function varargout = lambda(this, varargin)
      % LAMBDA usage: lambda() : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        class_wrapper(20, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.lambda');
    end

    function varargout = markdown(this, varargin)
      % MARKDOWN usage: markdown(KeyFormatter keyFormatter) : returns string
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyFormatter')
        varargout{1} = class_wrapper(21, this, varargin{:});
        return
      end
      % MARKDOWN usage: markdown() : returns string
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        varargout{1} = class_wrapper(22, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.markdown');
    end

    function varargout = print(this, varargin)
      % PRINT usage: print() : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 0
        class_wrapper(23, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.print');
    end

    function varargout = return_Point2Ptr(this, varargin)
      % RETURN_POINT2PTR usage: return_Point2Ptr(bool value) : returns Point2
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'logical')
        varargout{1} = class_wrapper(24, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_Point2Ptr');
    end

    function varargout = return_Test(this, varargin)
      % RETURN_TEST usage: return_Test(Test value) : returns Test
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = class_wrapper(25, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_Test');
    end

    function varargout = return_TestPtr(this, varargin)
      % RETURN_TESTPTR usage: return_TestPtr(Test value) : returns Test
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = class_wrapper(26, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_TestPtr');
    end

    function varargout = return_bool(this, varargin)
      % RETURN_BOOL usage: return_bool(bool value) : returns bool
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'logical')
        varargout{1} = class_wrapper(27, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_bool');
    end

    function varargout = return_double(this, varargin)
      % RETURN_DOUBLE usage: return_double(double value) : returns double
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = class_wrapper(28, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_double');
    end

    function varargout = return_field(this, varargin)
      % RETURN_FIELD usage: return_field(Test t) : returns bool
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = class_wrapper(29, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_field');
    end

    function varargout = return_int(this, varargin)
      % RETURN_INT usage: return_int(int value) : returns int
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = class_wrapper(30, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_int');
    end

    function varargout = return_matrix1(this, varargin)
      % RETURN_MATRIX1 usage: return_matrix1(Matrix value) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = class_wrapper(31, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_matrix1');
    end

    function varargout = return_matrix2(this, varargin)
      % RETURN_MATRIX2 usage: return_matrix2(Matrix value) : returns Matrix
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = class_wrapper(32, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_matrix2');
    end

    function varargout = return_pair(this, varargin)
      % RETURN_PAIR usage: return_pair(Vector v, Matrix A) : returns pair< Vector, Matrix >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double')
        [ varargout{1} varargout{2} ] = class_wrapper(33, this, varargin{:});
        return
      end
      % RETURN_PAIR usage: return_pair(Vector v) : returns pair< Vector, Matrix >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        [ varargout{1} varargout{2} ] = class_wrapper(34, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_pair');
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Test p1, Test p2) : returns pair< Test, Test >
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 2 && isa(varargin{1},'Test') && isa(varargin{2},'Test')
        [ varargout{1} varargout{2} ] = class_wrapper(35, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_ptrs');
    end

    function varargout = return_size_t(this, varargin)
      % RETURN_SIZE_T usage: return_size_t(size_t value) : returns size_t
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = class_wrapper(36, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_size_t');
    end

    function varargout = return_string(this, varargin)
      % RETURN_STRING usage: return_string(string value) : returns string
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'char')
        varargout{1} = class_wrapper(37, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_string');
    end

    function varargout = return_vector1(this, varargin)
      % RETURN_VECTOR1 usage: return_vector1(Vector value) : returns Vector
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = class_wrapper(38, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_vector1');
    end

    function varargout = return_vector2(this, varargin)
      % RETURN_VECTOR2 usage: return_vector2(Vector value) : returns Vector
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = class_wrapper(39, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.return_vector2');
    end

    function varargout = set_container(this, varargin)
      % SET_CONTAINER usage: set_container(vector<Test> container) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'std.vectorTest')
        class_wrapper(40, this, varargin{:});
        return
      end
      % SET_CONTAINER usage: set_container(vector<Test> container) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'std.vectorTest')
        class_wrapper(41, this, varargin{:});
        return
      end
      % SET_CONTAINER usage: set_container(vector<Test> container) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 1 && isa(varargin{1},'std.vectorTest')
        class_wrapper(42, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function Test.set_container');
    end


    function varargout = get.model_ptr(this)
        varargout{1} = class_wrapper(43, this);
        this.model_ptr = varargout{1};
    end

    function set.model_ptr(this, value)
        obj.model_ptr = value;
        class_wrapper(44, this, value);
    end

    function varargout = get.value(this)
        varargout{1} = class_wrapper(45, this);
        this.value = varargout{1};
    end

    function set.value(this, value)
        obj.value = value;
        class_wrapper(46, this, value);
    end

    function varargout = get.name(this)
        varargout{1} = class_wrapper(47, this);
        this.name = varargout{1};
    end

    function set.name(this, value)
        obj.name = value;
        class_wrapper(48, this, value);
    end
  end

  methods(Static = true)
  end
end
