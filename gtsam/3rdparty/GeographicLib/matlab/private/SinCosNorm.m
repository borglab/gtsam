function [sinx, cosx] = SinCosNorm(sinx, cosx)
%SINCOSNORM  Normalize sinx and cosx
%
%   [SINX, COSX] = SINCOSNORM(SINX, COSX) normalize SINX and COSX so that
%   SINX^2 + COSX^2 = 1.  SINX and COSX can be any shape.

  r = hypot(sinx, cosx);
  sinx = sinx ./ r;
  cosx = cosx ./ r;
end
