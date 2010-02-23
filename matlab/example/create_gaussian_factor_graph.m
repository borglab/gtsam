% Christan Potthast
% create linear factor graph

function lfg = create_gaussian_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n)

m = size(measurements,2);

% create linear factor graph
lfg = GaussianFactorGraph();

% Point2 at origin
origin = Point2;

% create prior for initial robot pose
model0_2 = SharedDiagonal([0.2;0.2]);
prior = Simulated2DPosePrior(origin,model0_2,1);
lf = prior.linearize(config);
lfg.push_back(lf);

% add prior for landmarks
model1000 = SharedDiagonal([1000;1000]);
for j = 1:n
    prior = Simulated2DPointPrior(origin,model1000,j);
    lf = prior.linearize(config); 
    lfg.push_back(lf);
end

% add odometry factors
odo_model = SharedDiagonal([odo_sigma;odo_sigma]);
for i = 2 : size(odometry,2)
    odo = Point2(odometry{i}(1),odometry{i}(2));
    nlf = Simulated2DOdometry(odo, odo_model, i-1, i);
    lf = nlf.linearize(config);
    lfg.push_back(lf);
end

% add measurement factors
measurement_model = SharedDiagonal([measurement_sigma;measurement_sigma]);
for k = 1 : size(measurements,2) 
    measurement = measurements{k};
    point = Point2(measurement.z(1),measurement.z(2));
    nlf = Simulated2DMeasurement(point, measurement_model, measurement.i, measurement.j);
    lf = nlf.linearize(config);
    lfg.push_back(lf);
end

