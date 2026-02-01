function pt = Point2(varargin)
    % Point2 shim
    if nargin == 2 && isa(varargin{1}, 'double')
        pt = [varargin{1} varargin{2}]';
    elseif nargin == 1
        pt = varargin{1};
    elseif nargin == 0
        pt = [0 0]';
    else
        error('Arguments do not match any overload of Point2 shim');
    end
end