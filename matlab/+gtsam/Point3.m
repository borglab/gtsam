function pt = Point3(varargin)
    % Point3 shim
    if nargin == 3 && isa(varargin{1}, 'double')
        pt = [varargin{1} varargin{2} varargin{3}]';
    elseif nargin == 1
        pt = varargin{1};
    else
        error('Arguments do not match any overload of MyVector3 constructor');
    end
end