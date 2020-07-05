function y = AngRound(x)
%ANGROUND  Round tiny values so that tiny values become zero.
%
%   y = ANGROUND(x) rounds x by adding and subtracting 1/16 to it if it is
%   small.  x can be any shape.

  z = 1/16;
  y = abs(x);
  y(y < z) = z - (z - y(y < z));
  y(x < 0) = -y(x < 0);
  y(x == 0) = 0;
end
