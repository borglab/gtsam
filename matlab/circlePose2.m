function values = circlePose2(numPoses, radius, symbolChar)
% circlePose2: generate a set of poses in a circle. This function
% returns those poses inside a gtsam.Values object, with sequential
% keys starting from 1. An optional character may be provided, which
% will be stored in the msb of each key (i.e. gtsam.Symbol).

if nargin<3,symbolChar=0x00;end
if nargin<2,radius=1.0;end
if nargin<1,numPoses=8;end

values = gtsam.Values;
theta = 0.0;
dtheta = 2*pi()/numPoses;
for i = 1:numPoses
    key = gtsam.Symbol(symbolChar, i);
    pose = gtsam.Pose2(radius*cos(theta), radius*sin(theta), pi()/2 + theta);
    values.insert(key, pose);
    theta = theta + dtheta;
end
