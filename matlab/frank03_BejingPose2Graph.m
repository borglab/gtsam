% frank03: create Beijing matrices in a cleaner way

load beijing.mat;
load beijing_angles.mat;
load beijing_graph.mat;
n=size(points,1);

% create config or load it from file
if 0
    load beijing_config.mat;
else
    config=Pose2Config();
    for j=1:n
        if mod(j,1000) == 0, fprintf(1, 'adding node %d to config\n', j); end   
        pose=Pose2(points(j,1),points(j,2),angles(j));
        key = sprintf('x%d', j);
        config.insert(key,pose);
    end
    save('beijing_config.mat','config');
end

sd = [0.25;0.25;0.01];

% Build factor graph for entire graph
graph = Pose2Graph;

% First add tree constraints
[I J] = find(tree);
for k=length(edge_order):-1:1
    edge = edge_order(k);
    if mod(k,1000) == 0, fprintf(1, 'simulating constraint %d\n', k); end   
    addSimulatedConstraint(points,angles,sd,I(edge),J(edge),graph);
end

% Then add remaining constraints C
C=G-tree;
[I J] = find(C);
for k=1:length(I)
    if mod(k,100) == 0, fprintf(1, 'simulating constraint %d\n', k); end   
    addSimulatedConstraint(points,angles,sd,I(k),J(k),graph);
end

% generate ordering
ordering = bottom_up_ordering(pred);