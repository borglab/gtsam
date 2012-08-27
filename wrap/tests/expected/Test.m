%-------Constructors-------
%Test()
%Test(double a, Matrix b)
%-------Methods-------
%arg_EigenConstRef(Matrix value)
%create_MixedPtrs()
%create_ptrs()
%print()
%return_Point2Ptr(bool value)
%return_Test(Test value)
%return_TestPtr(Test value)
%return_bool(bool value)
%return_double(double value)
%return_field(Test t)
%return_int(int value)
%return_matrix1(Matrix value)
%return_matrix2(Matrix value)
%return_pair(Vector v, Matrix A)
%return_ptrs(Test p1, Test p2)
%return_size_t(size_t value)
%return_string(string value)
%return_vector1(Vector value)
%return_vector2(Vector value)
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
      % arg_EigenConstRef  arg_EigenConstRef(Matrix value) : 
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
      % create_MixedPtrs  create_MixedPtrs() : 
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
      % create_ptrs  create_ptrs() : 
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
      % print  print() : 
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
      % return_Point2Ptr  return_Point2Ptr(bool value) : 
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
      % return_Test  return_Test(Test value) : 
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
      % return_TestPtr  return_TestPtr(Test value) : 
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
      % return_bool  return_bool(bool value) : 
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
      % return_double  return_double(double value) : 
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
      % return_field  return_field(Test t) : 
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
      % return_int  return_int(int value) : 
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
      % return_matrix1  return_matrix1(Matrix value) : 
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
      % return_matrix2  return_matrix2(Matrix value) : 
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
      % return_pair  return_pair(Vector v, Matrix A) : 
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
      % return_ptrs  return_ptrs(Test p1, Test p2) : 
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
      % return_size_t  return_size_t(size_t value) : 
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
      % return_string  return_string(string value) : 
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
      % return_vector1  return_vector1(Vector value) : 
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
      % return_vector2  return_vector2(Vector value) : 
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
