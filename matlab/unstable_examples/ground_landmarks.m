function [ values ] = ground_landmarks( nrPoints )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*;

values = Values;

x = -800+1600.*rand(nrPoints,1);
y = -800+1600.*rand(nrPoints,1);
z = 3 * rand(nrPoints,1);

for i=1:nrPoints
   values.insert(symbol('l',i),gtsam.Point3(x(i),y(i),z(i)));
    
end

end

