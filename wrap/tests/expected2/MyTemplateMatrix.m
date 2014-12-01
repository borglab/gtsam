%class MyTemplateMatrix, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
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
%templatedMethodPoint2(Point2 t) : returns gtsam::Point2
%templatedMethodPoint3(Point3 t) : returns gtsam::Point3
%templatedMethodVector(Vector t) : returns Vector
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
          my_ptr = geometry_wrapper(60, varargin{2});
        end
        base_ptr = geometry_wrapper(59, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = geometry_wrapper(61);
      else
        error('Arguments do not match any overload of MyTemplateMatrix constructor');
      end
      obj = obj@MyBase(uint64(5139824614673773682), base_ptr);
      obj.ptr_MyTemplateMatrix = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(62, obj.ptr_MyTemplateMatrix);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = accept_T(this, varargin)
      % ACCEPT_T usage: accept_T(Matrix value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        geometry_wrapper(63, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.accept_T');
      end
    end

    function varargout = accept_Tptr(this, varargin)
      % ACCEPT_TPTR usage: accept_Tptr(Matrix value) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        geometry_wrapper(64, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.accept_Tptr');
      end
    end

    function varargout = create_MixedPtrs(this, varargin)
      % CREATE_MIXEDPTRS usage: create_MixedPtrs() : returns pair< Matrix, Matrix >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(65, this, varargin{:});
    end

    function varargout = create_ptrs(this, varargin)
      % CREATE_PTRS usage: create_ptrs() : returns pair< Matrix, Matrix >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = geometry_wrapper(66, this, varargin{:});
    end

    function varargout = return_T(this, varargin)
      % RETURN_T usage: return_T(Matrix value) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(67, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.return_T');
      end
    end

    function varargout = return_Tptr(this, varargin)
      % RETURN_TPTR usage: return_Tptr(Matrix value) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(68, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.return_Tptr');
      end
    end

    function varargout = return_ptrs(this, varargin)
      % RETURN_PTRS usage: return_ptrs(Matrix p1, Matrix p2) : returns pair< Matrix, Matrix >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        [ varargout{1} varargout{2} ] = geometry_wrapper(69, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.return_ptrs');
      end
    end

    function varargout = templatedMethodMatrix(this, varargin)
      % TEMPLATEDMETHODMATRIX usage: templatedMethodMatrix(Matrix t) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        varargout{1} = geometry_wrapper(70, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethod');
      end
    end

    function varargout = templatedMethodPoint2(this, varargin)
      % TEMPLATEDMETHODPOINT2 usage: templatedMethodPoint2(Point2 t) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = geometry_wrapper(71, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethod');
      end
    end

    function varargout = templatedMethodPoint3(this, varargin)
      % TEMPLATEDMETHODPOINT3 usage: templatedMethodPoint3(Point3 t) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        varargout{1} = geometry_wrapper(72, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethod');
      end
    end

    function varargout = templatedMethodVector(this, varargin)
      % TEMPLATEDMETHODVECTOR usage: templatedMethodVector(Vector t) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = geometry_wrapper(73, this, varargin{:});
      else
        error('Arguments do not match any overload of function MyTemplateMatrix.templatedMethod');
      end
    end

  end

  methods(Static = true)
  end
end
