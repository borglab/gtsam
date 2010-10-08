dataSet = 'intel';
saveFiles = 0;
maxID = 0;
dim = 3;

[vertices,edges]=loadDataSet(dataSet,saveFiles);
[vert, ed]=cut2maxID(maxID, vertices, edges);

p0 = vert(1,2:end)';
s0 = diag([ed(1,6),ed(1,8),ed(1,9)]);

fprintf('Initial config...');
config=InitialConfig(ed,dim,p0);

fprintf('graph2FactorGraph...');
[A,b]=graph2FactorGraph(config,ed,dim,s0);

tic
for trial=1:100
    fprintf('Trial %d\n', trial);
    x = spqr_solve(A,b);
end
toc
