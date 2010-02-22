% Christian Potthast
% random walk from a robot

function pose = random_walk(initial, velocity, steps)

pose(:,1) = initial;
bearing = 1;


for step = 2:steps
    
    bearing =  bearing + 0.05*randn();
    
    pose(1,step) = pose(1,step-1) + sin(bearing) * velocity;
    pose(2,step) = pose(2,step-1) + cos(bearing) * velocity;
    
end



