function values = circlePose2(numPoses, radius, symbolChar)
% circlePose2 generates a set of poses in a circle. This function
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
for i = 1:numPoses
    key = gtsam.symbol(symbolChar, i-1);
    pose = gtsam.Pose2(radius*cos(theta), radius*sin(theta), pi()/2 + theta);
    values.insert(key, pose);
    theta = theta + dtheta;
end
