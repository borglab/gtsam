%% Initialize graph, initial estimate, and odometry noise
datafile = findExampleDataFile('sphere2500.txt');
model = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
[graph,initial] = load3D(datafile, model, true, 2500);
plot3DTrajectory(initial, 'g-', false); % Plot Initial Estimate

%% Read again, now with all constraints, and optimize
graph = load3D(datafile, model, false, 2500);
graph.add(NonlinearEqualityPose3(0, initial.atPose3(0)));
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely();
plot3DTrajectory(result, 'r-', false); axis equal;
