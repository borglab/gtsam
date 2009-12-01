% Christan Potthast
% create linear factor graph

function lfg = create_linear_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n)

m = size(measurements,2);

% create linear factor graph
lfg = GaussianFactorGraph();

% create prior for initial robot pose
prior = Point2Prior([0;0],0.2,'x1');
lf = prior.linearize(config);
lfg.push_back(lf);

% add prior for landmarks
for j = 1:n
    key = sprintf('l%d',j);
    prior = Point2Prior([0;0],1000,key);
    lf = prior.linearize(config); 
    lfg.push_back(lf);
end

% add measurement factors
for k = 1 : size(measurements,2) 
    measurement = measurements{k};
    i = sprintf('x%d',measurement.i);
    j = sprintf('l%d',measurement.j); 
    nlf = Simulated2DMeasurement(measurement.z, measurement_sigma, i, j);
    lf = nlf.linearize(config);
    lfg.push_back(lf);
end

% add odometry factors
for i = 2 : size(odometry,2)
    odo = odometry{i};
    p = sprintf('x%d',i-1);
    c = sprintf('x%d',i);
    nlf = Simulated2DOdometry(odo, odo_sigma, p, c);
    lf = nlf.linearize(config);
    lfg.push_back(lf);
end