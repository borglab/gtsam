function z = cvmgt(x, y, p)
%CVMGT  Conditional merge of two vectors
%
%   z = CVMGT(x, y, p) return a vector z whose elements are x if p is true
%   and y otherwise.  p, x, and y should be the same shape except that x
%   and y may be scalars.  CVMGT stands for conditional vector merge true
%   (an intrinsic function for the Cray fortran compiler).  It implements
%   the C++ statement
%
%     z = p ? x : y;

  z = zeros(size(p));
  if isscalar(x)
    z(p) = x;
  else
    z(p) = x(p);
  end
  if isscalar(y)
    z(~p) = y;
  else
    z(~p) = y(~p);
  end
end
