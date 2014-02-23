function y = AngRound(x)
%ANGROUND  Round tiny values so that tiny values become zero.
%
%   Y = ANGROUND(X) rounds X by adding and subtracting 1/16 to it if it is
%   small.  X and Y can be any shape.

  z = 1/16;
  y = abs(x);
  y(y < z) = z - (z - y(y < z));
  y(x < 0) = -y(x < 0);
end
