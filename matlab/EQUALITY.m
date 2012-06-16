function EQUALITY(name,A,B,tol)
% test equality of two vectors/matrices up to tolerance

if nargin<4,tol=1e-9;end

assertion = size(A)==size(B);
if assertion
    assertion = all(abs(A-B)<tol);
end
if (assertion~=1)
  error(['EQUALITY ' name ' fails']);
end
