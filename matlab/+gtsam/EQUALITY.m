function EQUALITY(name,expected,actual,tol)
% test equality of two vectors/matrices up to tolerance

if nargin<4,tol=1e-9;end

sameSize = size(expected)==size(actual);
if all(sameSize)
    equal = abs(expected-actual)<tol;
    if ~all(equal(:))
        warning(['EQUALITY ' name ' fails']);
        expected
        actual
        abs(expected-actual)
    end
else
    warning(['EQUALITY ' name ' fails: non-matching size']);
    size(expected)
    size(actual)
end
