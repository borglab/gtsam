function taup = taupf(tau, e2)
%TAUPF   tan(chi)
%
%   TAUPF(tau, e2) returns tangent of chi in terms of tau the tangent of
%   phi.  e2, the square of the eccentricity, is a scalar; taup can be any
%   shape.

  tau1 = hypot(1, tau);
  sig = sinh( eatanhe( tau ./ tau1, e2 ) );
  taup = hypot(1, sig) .* tau - sig .* tau1;
end
