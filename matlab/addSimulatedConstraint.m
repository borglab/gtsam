function addSimulatedConstraint(points,angles,sd,id1,id2,graph)
% addSimulatedConstraint: create a simulated measurement with noise
% standard deviations sd and add it to graph

key1 = sprintf('x%d', id1);
key2 = sprintf('x%d', id2);

% ground truth
delta_x = points(id1,:) - points(id2,:);
delta_angle = angles(id1) - angles(id2);
noisy = Pose2(delta_x(1) + sd(1)*randn, delta_x(2) + sd(2)*randn, delta_angle + sd(3)*randn);

% create factor
factor=Pose2Factor(key1,key2,noisy,diag(sd.*sd));

% add it to the graph
graph.push_back(factor);
