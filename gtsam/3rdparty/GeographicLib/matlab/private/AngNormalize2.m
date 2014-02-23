function x = AngNormalize2(x)
%ANGNORMALIZE2  Reduce any angle to range [-180, 180)
%
%   X = ANGNORMALIZE(X) reduces arbitrary angles to the range [-180, 180).
%   X can be any shape.

  x = AngNormalize(mod(x, 360));
end
