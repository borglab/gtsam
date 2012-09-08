function values = circlePose3(numPoses, radius, symbolChar)
% circlePose3 generates a set of poses in a circle. This function
% returns those poses inside a gtsam.Values object, with sequential
% keys starting from 0. An optional character may be provided, which
% will be stored in the msb of each key (i.e. gtsam.Symbol).
%
% We use aerospace/navlab convention, X forward, Y right, Z down
% First pose will be at (R,0,0)
% ^y   ^ X
% |    |
% z-->xZ--> Y  (z pointing towards viewer, Z pointing away from viewer)
% Vehicle at p0 is looking towards y axis (X-axis points towards world y)

if nargin<3,symbolChar=0;end
if nargin<2,radius=1.0;end
if nargin<1,numPoses=8;end

% Force symbolChar to be a single character
symbolChar = char(symbolChar);
symbolChar = symbolChar(1);

values = gtsam.Values;
theta = 0.0;
dtheta = 2*pi()/numPoses;
gRo = gtsam.Rot3([0, 1, 0 ; 1, 0, 0 ; 0, 0, -1]);
for i = 1:numPoses
    key = gtsam.symbol(symbolChar, i-1);
    gti = gtsam.Point3(radius*cos(theta), radius*sin(theta), 0);
    oRi = gtsam.Rot3.Yaw(-theta);  % negative yaw goes counterclockwise, with Z down !
    gTi = gtsam.Pose3(gRo.compose(oRi), gti);
    values.insert(key, gTi);
    theta = theta + dtheta;
end
