%class HessianFactor, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Constructors-------
%HessianFactor(KeyVector js, vector<Matrix> Gs, vector<Vector> gs, double f)
%
classdef HessianFactor < gtsam.GaussianFactor
  properties
    ptr_HessianFactor = 0
  end
  methods
    function obj = HessianFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = class_wrapper(80, varargin{2});
        end
        base_ptr = class_wrapper(79, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'gtsam.KeyVector') && isa(varargin{2},'std.vectordouble') && isa(varargin{3},'std.vectordouble') && isa(varargin{4},'double')
        [ my_ptr, base_ptr ] = class_wrapper(81, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of HessianFactor constructor');
      end
      obj = obj@gtsam.GaussianFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_HessianFactor = my_ptr;
    end

    function delete(obj)
      class_wrapper(82, obj.ptr_HessianFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
