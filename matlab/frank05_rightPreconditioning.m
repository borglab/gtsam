% frank05: show right pre-conditioning
% first run frank03 or other script to generate graph, config, and ordering

% linearize the non-linear factor graph
tic
LFG = graph.linearize_(config);
toc
tic
ijs = LFG.sparse(ordering);
A = sparse(ijs(1,:),ijs(2,:),ijs(3,:)); 
toc
figure(1)
spy(A);

% isolate the spanning tree part
A1=A(1:3*nnz(tree),:);
% add prior
figure(2)
spy(A1)

% calculate R1
tic
R1 = qr(A1,0);
toc
figure(3)
spy(R1)

% calculate R1
tic
R1 = chol(A1'*A1);
toc
figure(3)
spy(R1)

% calculate the entire R factor (expensive)
tic
R = qr(A,0);
toc
figure(4)
spy(R)

