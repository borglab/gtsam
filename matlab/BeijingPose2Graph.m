load beijing.mat;
load beijing_angles.mat;
load beijing_graph.mat;
%load beijing_config.mat;

cov = [ 0.25, 0, 0; 0, 0.25, 0; 0, 0, 0.01];


factors = Pose2Graph;
factors2 = Pose2Graph;
ord = Ordering();

for i=1:50%length(ways)
    if mod(i,50) == 0
        fprintf(1, 'processing way %d\n', i);
    end
    for j=1:length(ways{i})-1
        id1 = ways{i}(j);
        id2 = ways{i}(j+1);
        key1 = sprintf('x%d', id1);
        key2 = sprintf('x%d', id2);

        delta_x = points(id1,:) - points(id2,:);
        delta_angle = angles(id1) - angles(id2);
        measured = Pose2(delta_x(1), delta_x(2), delta_angle);

        if pred(id1) == id2 || pred(id2) == id1 %% in the spanning tree
            factor=Pose2Factor(key1,key2,measured, cov);
            factors.push_back(factor);
            ord.push_back(key1);
            ord.push_back(key2);
        else %% not in the spanning tree
            factors2.push_back(Pose2Factor(key1,key2,measured, cov));
        end
    end
end

ord.unique();
config=Pose2Config();
n=size(points,1);
for j=1:n
    pose=Pose2(points(j,1),points(j,2),angles(j));
     key = sprintf('x%d', j);
    config.insert(key,pose);
       if mod(j,50) == 0
           key
    end
end
save('beijing_config.mat','config');

ord2=factors.getOrdering_();
fprintf(1,'linearize...');
LFG=factors.linearize_(config);
fprintf(1,'linearize done!');
ijs=LFG.sparse(ord2);
fprintf(1,'sparse done');
A=sparse(ijs(1,:),ijs(2,:),ijs(3,:)); 
spy(A);
%save('beijing_factors.mat', 'factors');

