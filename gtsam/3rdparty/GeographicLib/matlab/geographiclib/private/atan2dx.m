function z = atan2dx(y, x)
%ATAN2DX  Compute 2 argument arctangent with result in degrees
%
%   z = ATAN2DX(y, x) compute atan2(y, x) with result in degrees in
%   (-180,180] and quadrant symmetries enforced.  x and y must have
%   compatible shapes.

  persistent octavep
  if isempty(octavep)
    octavep = exist('OCTAVE_VERSION', 'builtin') ~= 0;
  end
  if ~octavep
    % MATLAB implements symmetries already
    z = atan2d(y, x);
  else
    q1 = abs(y) > abs(x); q2 = ones(size(q1));
    x = q2 .* x; y = q2 .* y;           % expand x, y if necessary
    t = y(q1); y(q1) = x(q1); x(q1) = t;
    q2 = x < 0; x(q2) = -x(q2);
    q = 2 * q1 + q2;
    z = atan2(y, x) * (180 / pi);       % z in [-45, 45]
    % t = q == 0;        z(t) =    0 + z(t);
    t = q == 1 & y >= 0; z(t) =  180 - z(t);
    t = q == 1 & y <  0; z(t) = -180 - z(t);
    t = q == 2;          z(t) =   90 - z(t);
    t = q == 3;          z(t) =  -90 + z(t);
  end
end
