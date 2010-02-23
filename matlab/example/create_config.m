% Christian Potthast
% create a configuration

function config = create_config(n,m)

config = Simulated2DConfig();
origin = Point2;

for i = 1:m
    config.insertPose(i, origin); 
end

for j = 1:n
    config.insertPoint(j, origin);
end