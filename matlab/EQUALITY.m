function EQUALITY(name,expected,actual,tol)
% test equality of two vectors/matrices up to tolerance

if nargin<4,tol=1e-9;end

assertion = size(expected)==size(actual);
if assertion
    assertion = all(abs(expected-actual)<tol);
end
if (assertion~=1)
  warning(['EQUALITY ' name ' fails']);
  expected
  actual
end
