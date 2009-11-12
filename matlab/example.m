% Set up a small SLAM example in MATLAB
% Authors: Christian Potthast, Frank Dellaert

clear;

n = 1000;
m = 200;

% Set up the map 
    map = create_random_landmarks(n,[1000,1000]);
figure(1);clf;
plot(map(1,:), map(2,:),'g.'); hold on;

% have the robot move in this world
trajectory = random_walk([0.1,0.1],5,m);
plot(trajectory(1,:),trajectory(2,:),'b+');
axis([0 1000 0 1000]);axis square;

% Check visibility and plot this on the problem figure
visibility = create_visibility(map, trajectory,50);
gplot(visibility,[map trajectory]');
figure(2);clf;
spy(visibility)

% simulate the measurements
measurement_sigma = 1;
odo_sigma = 0.1;
[measurements, odometry] = simulate_measurements(map, trajectory, visibility, measurement_sigma, odo_sigma);

% create a configuration of all zeroes
config = create_config(n,m);

% create the factor graph
linearFactorGraph = create_linear_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n);

% create an ordering
ord = create_ordering(n,m);

% show the matrix
figure(3); clf;
[A_dense,b] = linearFactorGraph.matrix(ord);
%spy(A);
 A=sparse(A_dense);
% eliminate with that ordering
ck = cputime;
BayesNet = linearFactorGraph.eliminate(ord);
time_gtsam = cputime - ck

ckqr = cputime;
R = qr(A);
time_qr = cputime - ckqr


%time_gtsam=[time_gtsam,(cputime-ck)]
% show the eliminated matrix
figure(4); clf;
[R,d] = BayesNet.matrix();
spy(R);

% optimize in the BayesNet
optimal = BayesNet.optimize;

% plot the solution
figure(5);clf; 
plot_config(optimal,n,m);hold on
plot(trajectory(1,:),trajectory(2,:),'b+');
plot(map(1,:), map(2,:),'g.');
axis([0 1000 0 1000]);axis square;
