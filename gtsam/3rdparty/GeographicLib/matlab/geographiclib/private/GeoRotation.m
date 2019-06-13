function M = GeoRotation(sphi, cphi, slam, clam)
%GEOROTATION The rotation from geodetic to geocentric
%
%   M = GeoRotation(sphi, cphi, slam, clam)
%
%   sphi, cphi, slam, clam must all have the same shape, S.  M has the
%   shape [3, 3, S].

% This rotation matrix is given by the following quaternion operations
% qrot(lam, [0,0,1]) * qrot(phi, [0,-1,0]) * [1,1,1,1]/2
% or
% qrot(pi/2 + lam, [0,0,1]) * qrot(-pi/2 + phi , [-1,0,0])
% where
% qrot(t,v) = [cos(t/2), sin(t/2)*v[1], sin(t/2)*v[2], sin(t/2)*v[3]]
  S = size(sphi);
  M = zeros(9, prod(S));
  sphi = sphi(:); cphi = cphi(:);
  slam = slam(:); clam = clam(:);
  % Local X axis (east) in geocentric coords
  M(1,:) = -slam;         M(2,:) =  clam;         M(3,:) = 0;
  % Local Y axis (north) in geocentric coords
  M(4,:) = -clam .* sphi; M(5,:) = -slam .* sphi; M(6,:) = cphi;
  % Local Z axis (up) in geocentric coords
  M(7,:) =  clam .* cphi; M(8,:) =  slam .* cphi; M(9,:) = sphi;
  M = reshape(M, [3, 3, S]);
end
