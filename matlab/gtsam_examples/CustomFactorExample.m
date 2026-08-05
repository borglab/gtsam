import gtsam.*;

X = simulate_car();
fprintf('Simulated car trajectory: [%s]\n', num2str(X));

add_noise = true;

% GPS measurements
sigma_gps = 3.0;
gps_noise_model = noiseModel.Isotropic.Sigma(1, sigma_gps);
gps_sampler = Sampler(gps_noise_model, 42);
gps_measurements = zeros(1, 5);
for k=1:5
    gps_measurements(k) = X(k) + (add_noise * gps_sampler.sample());
end

% Odometry measurements
sigma_odo = 0.1;
odo_noise_model = noiseModel.Isotropic.Sigma(1, sigma_odo);
odo_sampler = Sampler(odo_noise_model, 42);
odometry_measurements = zeros(1, 4);
X_diff = diff(X);
for k=1:4
    odometry_measurements(k) = X_diff(k) + (add_noise * odo_sampler.sample());
end

% Landmark measurements:
sigma_lm = 1;
lm_noise_model = noiseModel.Isotropic.Sigma(1, sigma_lm);
lm_sampler = Sampler(lm_noise_model, 42);

lm_0 = 5.0;
landmark0_measurement = X(1) - lm_0 + (add_noise * lm_sampler.sample());
lm_3 = 28.0;
landmark3_measurement = X(4) - lm_3 + (add_noise * lm_sampler.sample());

unknown = [];
for k=0:4
    unknown = [unknown symbol('x', k)];
end

% 1. Result with only GPS
factor_graph = NonlinearFactorGraph;
factors = {}; % Keep alive
for k=1:5
    error_func = make_gps_callback(gps_measurements(k));
    gps_factor = CustomFactor(gps_noise_model, unknown(k), error_func);
    factor_graph.add(gps_factor);
    factors{end+1} = gps_factor;
end

v = Values;
for i=1:5
    v.insert(unknown(i), [0.0]);
end

params = GaussNewtonParams;
result = GaussNewtonOptimizer(factor_graph, v, params).optimize();

vals = zeros(1, 5);
for i=1:5, vals(i) = result.atVector(unknown(i)); end
fprintf('Result with only GPS: [%s], Error: %f\n', num2str(round(vals, 2)), factor_graph.error(result));

% 2. Result with GPS+Odometry
for k=1:4
    error_func = make_odom_callback(odometry_measurements(k));
    odometry_factor = CustomFactor(odo_noise_model, [unknown(k), unknown(k + 1)], error_func);
    factor_graph.add(odometry_factor);
    factors{end+1} = odometry_factor;
end

result = GaussNewtonOptimizer(factor_graph, v, params).optimize();
vals = zeros(1, 5);
for i=1:5, vals(i) = result.atVector(unknown(i)); end
fprintf('Result with GPS+Odometry: [%s], Error: %f\n', num2str(round(vals, 2)), factor_graph.error(result));

% 3. Result with GPS+Odometry+Landmark
error_func0 = make_lm_callback(lm_0 + landmark0_measurement);
factor0 = CustomFactor(lm_noise_model, unknown(1), error_func0);
factor_graph.add(factor0);
factors{end+1} = factor0;

error_func3 = make_lm_callback(lm_3 + landmark3_measurement);
factor3 = CustomFactor(lm_noise_model, unknown(4), error_func3);
factor_graph.add(factor3);
factors{end+1} = factor3;

result = GaussNewtonOptimizer(factor_graph, v, params).optimize();
vals = zeros(1, 5);
for i=1:5, vals(i) = result.atVector(unknown(i)); end
fprintf('Result with GPS+Odometry+Landmark: [%s], Error: %f\n', num2str(round(vals, 2)), factor_graph.error(result));

%% Callback Factory Helpers
function f = make_gps_callback(measurement)
    f = @(varargin) error_gps(measurement, varargin{:});
end

function f = make_odom_callback(measurement)
    f = @(varargin) error_odom(measurement, varargin{:});
end

function f = make_lm_callback(measurement)
    f = @(varargin) error_lm(measurement, varargin{:});
end

%% Helper Functions
function x = simulate_car()
    x0 = 0;
    dt = 0.25;
    v = 144 * 1000 / 3600;
    x = x0 + v * dt * (0:4);
end

function varargout = error_gps(measurement, this, values)
    keys = this.keys();
    estimate = values.atVector(keys.at(0));
    residual = estimate - measurement;
    if nargout > 1
        varargout{1} = residual;
        varargout{2} = {1};
    else
        varargout{1} = residual;
    end
end

function varargout = error_odom(measurement, this, values)
    keys = this.keys();
    pos1 = values.atVector(keys.at(0));
    pos2 = values.atVector(keys.at(1));
    residual = (pos2 - pos1) - measurement;
    if nargout > 1
        varargout{1} = residual;
        varargout{2} = {-1, 1};
    else
        varargout{1} = residual;
    end
end

function varargout = error_lm(measurement, this, values)
    keys = this.keys();
    pos = values.atVector(keys.at(0));
    residual = pos - measurement;
    if nargout > 1
        varargout{1} = residual;
        varargout{2} = {1}; % Jacobian w.r.t. x
    else
        varargout{1} = residual;
    end
end
