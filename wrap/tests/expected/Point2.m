%class Point2, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Point2()
%Point2(double x, double y)
%
%-------Methods-------
%argChar(char a) : returns void
%argUChar(unsigned char a) : returns void
%dim() : returns int
%returnChar() : returns char
%vectorConfusion() : returns VectorNotEigen
%x() : returns double
%y() : returns double
%
classdef Point2 < handle
  properties
    ptr_Point2 = 0
  end
  methods
    function obj = Point2(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        geometry_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = geometry_wrapper(1);
      elseif nargin == 2 && isa(varargin{1},'double') && isa(varargin{2},'double')
        my_ptr = geometry_wrapper(2, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of Point2 constructor');
      end
      obj.ptr_Point2 = my_ptr;
    end

    function delete(obj)
      geometry_wrapper(3, obj.ptr_Point2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = argChar(this, varargin)
      % ARGCHAR usage: argChar(char a) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % argChar(char a)
      if length(varargin) == 1 && isa(varargin{1},'char')
        geometry_wrapper(4, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.argChar');
      end
    end

    function varargout = argUChar(this, varargin)
      % ARGUCHAR usage: argUChar(unsigned char a) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % argUChar(unsigned char a)
      if length(varargin) == 1 && isa(varargin{1},'char')
        geometry_wrapper(5, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.argUChar');
      end
    end

    function varargout = dim(this, varargin)
      % DIM usage: dim() : returns int
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % dim()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(6, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.dim');
      end
    end

    function varargout = returnChar(this, varargin)
      % RETURNCHAR usage: returnChar() : returns char
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % returnChar()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(7, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.returnChar');
      end
    end

    function varargout = vectorConfusion(this, varargin)
      % VECTORCONFUSION usage: vectorConfusion() : returns VectorNotEigen
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % vectorConfusion()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(8, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.vectorConfusion');
      end
    end

    function varargout = x(this, varargin)
      % X usage: x() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % x()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(9, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.x');
      end
    end

    function varargout = y(this, varargin)
      % Y usage: y() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      % 
      % Method Overloads
      % y()
      if length(varargin) == 0
        varargout{1} = geometry_wrapper(10, this, varargin{:});
      else
        error('Arguments do not match any overload of function Point2.y');
      end
    end

  end

  methods(Static = true)
  end
end
