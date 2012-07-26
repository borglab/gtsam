function values = circlePose2(numPoses, radius, symbolChar)
% circlePose3: generate a set of poses in a circle. This function
% returns those poses inside a gtsam.Values object, with sequential
% keys starting from 1. An optional character may be provided, which
% will be stored in the msb of each key (i.e. gtsam.Symbol).
%
% We use aerospace/navlab convention, X forward, Y right, Z down
% First pose will be at (R,0,0)
% ^y   ^ X
% |    |
% z-->xZ--> Y  (z pointing towards viewer, Z pointing away from viewer)
% Vehicle at p0 is looking towards y axis (X-axis points towards world y)

if nargin<3,symbolChar=0x00;end
if nargin<2,radius=1.0;end
if nargin<1,numPoses=8;end

values = gtsam.Values;
theta = 0.0;
dtheta = 2*pi()/numPoses;
gR0 = gtsam.Rot3(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
for i = 1:numPoses
    key = gtsam.Symbol(symbolChar, i);
    gti = Point3(radius*cos(theta), radius*sin(theta), 0);
    _0Ri = gtsam.Rot3.yaw(-theta);  % negative yaw goes counterclockwise, with Z down !
    gTi = gtsam.Pose3(gR0.compose(_0Ri), gti);
    values.insert(key, gTi);
    theta = theta + dtheta;
end
