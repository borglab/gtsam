% frank04: show matrices associated with Beijing simulation

load beijing.mat;
load beijing_angles.mat;
load beijing_graph.mat;

% put spanning tree 'tree' in correct order
T = tree(node_order,node_order);

% get loop closing constraints
C=G(node_order,node_order)-T;

close all

% plot on map
figure
gplot(C,points(node_order,:),'r')
hold on
gplot(T,points(node_order,:),'k')
axis equal

% show spanning tree, original graph, and loop closures
figure
spy(T,'k');
hold on
spy(C,'r');

figure
spy(T);

figure
spy(C);


