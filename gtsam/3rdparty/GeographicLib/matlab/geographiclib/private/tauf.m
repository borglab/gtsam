function tau = tauf(taup, e2)
%TAUF   tan(phi)
%
%   TAUF(taup, e2) returns tangent of phi in terms of taup the tangent of
%   chi.  e2, the square of the eccentricity, is a scalar; taup can be any
%   shape.

  numit = 5;
  e2m = 1 - e2;
  tau = taup / e2m;
  stol = 0.1 * sqrt(eps) * max(1, abs(taup));
  g = isfinite(tau);
  for i = 1 : numit
    if ~any(g), break, end
    tau1 = hypot(1, tau);
    sig = sinh( eatanhe( tau ./ tau1, e2 ) );
    taupa = hypot(1, sig) .* tau - sig .* tau1;
    dtau = (taup - taupa) .* (1 + e2m .* tau.^2) ./ ...
           (e2m * tau1 .* hypot(1, taupa));
    tau(g) = tau(g) + dtau(g);
    g = g & abs(dtau) >= stol;
  end
end
