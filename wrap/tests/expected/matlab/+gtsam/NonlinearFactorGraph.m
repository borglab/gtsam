%class NonlinearFactorGraph, see Doxygen page for details
%at https://gtsam.org/doxygen/
%
%-------Methods-------
%addPriorPinholeCameraCal3Bundler(size_t key, PinholeCamera<Cal3Bundler> prior, Base noiseModel) : returns void
%
classdef NonlinearFactorGraph < handle
  properties
    ptr_gtsamNonlinearFactorGraph = 0
  end
  methods
    function obj = NonlinearFactorGraph(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        special_cases_wrapper(0, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.NonlinearFactorGraph constructor');
      end
      obj.ptr_gtsamNonlinearFactorGraph = my_ptr;
    end

    function delete(obj)
      special_cases_wrapper(1, obj.ptr_gtsamNonlinearFactorGraph);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = addPriorPinholeCameraCal3Bundler(this, varargin)
      % ADDPRIORPINHOLECAMERACAL3BUNDLER usage: addPriorPinholeCameraCal3Bundler(size_t key, PinholeCamera<Cal3Bundler> prior, Base noiseModel) : returns void
      % Doxygen can be found at https://gtsam.org/doxygen/
      if length(varargin) == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.PinholeCameraCal3Bundler') && isa(varargin{3},'gtsam.noiseModel.Base')
        special_cases_wrapper(2, this, varargin{:});
        return
      end
      error('Arguments do not match any overload of function gtsam.NonlinearFactorGraph.addPriorPinholeCameraCal3Bundler');
    end

  end

  methods(Static = true)
  end
end
