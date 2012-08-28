%-------Constructors-------
%TEST()
%TEST(double a, Matrix b)
% 
%-------Methods-------
%ARG_EIGENCONSTREF(Matrix value) : returns void
%CREATE_MIXEDPTRS() : returns pair< Test, SharedTest >
%CREATE_PTRS() : returns pair< SharedTest, SharedTest >
%PRINT() : returns void
%RETURN_POINT2PTR(bool value) : returns Point2
%RETURN_TEST(Test value) : returns Test
%RETURN_TESTPTR(Test value) : returns Test
%RETURN_BOOL(bool value) : returns bool
%RETURN_DOUBLE(double value) : returns double
%RETURN_FIELD(Test t) : returns bool
%RETURN_INT(int value) : returns int
%RETURN_MATRIX1(Matrix value) : returns Matrix
%RETURN_MATRIX2(Matrix value) : returns Matrix
%RETURN_PAIR(Vector v, Matrix A) : returns pair< Vector, Matrix >
%RETURN_PTRS(Test p1, Test p2) : returns pair< SharedTest, SharedTest >
%RETURN_SIZE_T(size_t value) : returns size_t
%RETURN_STRING(string value) : returns string
%RETURN_VECTOR1(Vector value) : returns Vector
%RETURN_VECTOR2(Vector value) : returns Vector
% 
%-------Static Methods-------
%
%For more detailed documentation on GTSAM go to our Doxygen page, which can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
classdef Test < handle
  properties
    ptr_Test = 0
  end
  methods
    function obj = Test(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(17, my_ptr);
      elseif nargin == 0
        my_ptr = geometry_wrapper(18);
      elseif nargin == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        my_ptr = geometry_wrapper(19, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of Test constructor');
      end
      obj.ptr_Test = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(20, obj.ptr_Test);
    end

    function display(obj), obj.print(''); end

    function disp(obj), obj.display; end

    function varargout = arg_EigenConstRef(this, varargin)
      % ARG_EIGENCONSTREF usage: ARG_EIGENCONSTREF(Matrix value) : returns void

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % arg_EigenConstRef(Matrix value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        geometry_wrapper(21, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.arg_EigenConstRef');
      end
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: CREATE_MIXEDPTRS() : returns pair< Test, SharedTest >

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % create_MixedPtrs()
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = geometry_wrapper(22, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.create_MixedPtrs');
      end
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: CREATE_PTRS() : returns pair< SharedTest, SharedTest >

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % create_ptrs()
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = geometry_wrapper(23, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.create_ptrs');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: PRINT() : returns void

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % print()
      if length(varargin) == 0
        geometry_wrapper(24, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.print');
      end
    end

    function varargout = return_Point2Ptr(this, varargin)
      % RETURN_POINT2PTR usage: RETURN_POINT2PTR(bool value) : returns Point2

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_Point2Ptr(bool value)
      if length(varargin) == 1 && isa(varargin{1},'logical')
        varargout{1} = geometry_wrapper(25, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_Point2Ptr');
      end
    end

    function varargout = return_Test(this, varargin)
      % RETURN_TEST usage: RETURN_TEST(Test value) : returns Test

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_Test(Test value)
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = geometry_wrapper(26, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_Test');
      end
    end

    function varargout = return_TestPtr(this, varargin)
      % RETURN_TESTPTR usage: RETURN_TESTPTR(Test value) : returns Test

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_TestPtr(Test value)
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = geometry_wrapper(27, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_TestPtr');
      end
    end

    function varargout = return_bool(this, varargin)
      % RETURN_BOOL usage: RETURN_BOOL(bool value) : returns bool

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_bool(bool value)
      if length(varargin) == 1 && isa(varargin{1},'logical')
        varargout{1} = geometry_wrapper(28, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_bool');
      end
    end

    function varargout = return_double(this, varargin)
      % RETURN_DOUBLE usage: RETURN_DOUBLE(double value) : returns double

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_double(double value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(29, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_double');
      end
    end

    function varargout = return_field(this, varargin)
      % RETURN_FIELD usage: RETURN_FIELD(Test t) : returns bool

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_field(Test t)
      if length(varargin) == 1 && isa(varargin{1},'Test')
        varargout{1} = geometry_wrapper(30, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_field');
      end
    end

    function varargout = return_int(this, varargin)
      % RETURN_INT usage: RETURN_INT(int value) : returns int

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_int(int value)
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = geometry_wrapper(31, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_int');
      end
    end

    function varargout = return_matrix1(this, varargin)
      % RETURN_MATRIX1 usage: RETURN_MATRIX1(Matrix value) : returns Matrix

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_matrix1(Matrix value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(32, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_matrix1');
      end
    end

    function varargout = return_matrix2(this, varargin)
      % RETURN_MATRIX2 usage: RETURN_MATRIX2(Matrix value) : returns Matrix

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_matrix2(Matrix value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(33, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_matrix2');
      end
    end

    function varargout = return_pair(this, varargin)
      % RETURN_PAIR usage: RETURN_PAIR(Vector v, Matrix A) : returns pair< Vector, Matrix >

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_pair(Vector v, Matrix A)
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        [ varargout{1} varargout{2} ] = geometry_wrapper(34, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_pair');
      end
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: RETURN_PTRS(Test p1, Test p2) : returns pair< SharedTest, SharedTest >

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_ptrs(Test p1, Test p2)
      if length(varargin) == 2 && isa(varargin{1},'Test') && isa(varargin{2},'Test')
        [ varargout{1} varargout{2} ] = geometry_wrapper(35, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_ptrs');
      end
    end

    function varargout = return_size_t(this, varargin)
      % RETURN_SIZE_T usage: RETURN_SIZE_T(size_t value) : returns size_t

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_size_t(size_t value)
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = geometry_wrapper(36, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_size_t');
      end
    end

    function varargout = return_string(this, varargin)
      % RETURN_STRING usage: RETURN_STRING(string value) : returns string

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_string(string value)
      if length(varargin) == 1 && isa(varargin{1},'char')
        varargout{1} = geometry_wrapper(37, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_string');
      end
    end

    function varargout = return_vector1(this, varargin)
      % RETURN_VECTOR1 usage: RETURN_VECTOR1(Vector value) : returns Vector

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_vector1(Vector value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(38, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_vector1');
      end
    end

    function varargout = return_vector2(this, varargin)
      % RETURN_VECTOR2 usage: RETURN_VECTOR2(Vector value) : returns Vector

      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % return_vector2(Vector value)
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(39, this, varargin{:});
      else
        error('Arguments do not match any overload of function Test.return_vector2');
      end
    end

  end

  methods(Static = true)
  end
end
