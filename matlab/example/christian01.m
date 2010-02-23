% Set up a small SLAM example
% Christian Potthast, Frank Dellaert

clear;
close all;

n = 100;
m = 20;

% have the robot move in this world
trajectory = random_walk([0.1,0.1],5,m);
plot(trajectory(1,:),trajectory(2,:),'b+');
axis([0 100 0 100]);axis square;

% Set up the map 
mappingArea=max(trajectory,[],2);
map = create_random_landmarks(n, mappingArea);
figure(1);clf;
plot(map(1,:), map(2,:),'g.'); hold on;
axis([0 mappingArea(1) 0 mappingArea(2)]);axis square;

% Check visibility and plot this on the problem figure
visibility = create_visibility(map, trajectory,10);
gplot(visibility,[map trajectory]');

% simulate the measurements
measurement_sigma = 1;
odo_sigma = 0.1;
[measurements, odometry] = simulate_measurements(map, trajectory, visibility, measurement_sigma, odo_sigma);

% create a configuration of all zeroes
config = create_config(n,m);

% create the factor graph
factorGraph = create_gaussian_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n);

% create an ordering
ord = create_ordering(n,m);

% show the matrix
figure(2); clf;
A = factorGraph.matrix(ord);
spy(A);

% optimizing a BayesNet is not possible from MATLAB as
% GaussianBayesNet is a typedef not a real class :-(
% BayesNet = factorGraph.eliminate_(ord);
% optimal = BayesNet.optimize;

% However, we can call the optimize_ method of a GaussianFactorGraph
optimal = factorGraph.optimize_(ord);

% plot the solution
figure(3);clf; 
plot_config(optimal,n,m);hold on
plot(trajectory(1,:),trajectory(2,:),'b+');
plot(map(1,:), map(2,:),'g.');
axis([0 mappingArea(1) 0 mappingArea(2)]);axis square;
