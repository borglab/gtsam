function z = cvmgt(x, y, p)
%CVMGT  Conditional merge of two vectors
%
%   Z = CVMGT(X, Y, P) return a vector Z whose elements are X if P is true
%   and Y otherwise.  P, X, and Y should be the same shape except that X
%   and Y may be scalars.  CVMGT stands for conditional vector merge true
%   (an intrinsic function for the Cray fortran compiler).  It implements
%   the C++ statement
%
%     Z = P ? X : Y;

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
