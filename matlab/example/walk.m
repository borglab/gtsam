% Christian Potthast
% random walk from a robot

function pose = walk(initial, velocity, steps)

pose(:,1) = initial;
bearing = 0.7854; % 45?


for step = 2:steps
    
    %bearing =  bearing + 0.05;
    
    pose(1,step) = pose(1,step-1) + sin(bearing) * velocity;
    pose(2,step) = pose(2,step-1) + cos(bearing) * velocity;
    
end



