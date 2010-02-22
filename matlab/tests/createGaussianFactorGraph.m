% create a linear factor graph
% The non-linear graph above evaluated at NoisyConfig
function fg = createGaussianFactorGraph()

c = createNoisyConfig(); 

% Create
fg = GaussianFactorGraph;

% Create shared Noise model
unit2 = SharedDiagonal([1;1]);

% prior on x1
I=eye(2);
f1 = GaussianFactor('x1', 10*I, [-1;-1], unit2);
fg.push_back(f1);

% odometry between x1 and x2
f2 = GaussianFactor('x1', -10*I, 'x2', 10*I, [2;-1], unit2);
fg.push_back(f2);

% measurement between x1 and l1
f3 = GaussianFactor('x1', -5*I, 'l1', 5*I, [0;1], unit2);
fg.push_back(f3);

% measurement between x2 and l1
f4 = GaussianFactor('x2', -5*I, 'l1', 5*I, [-1;1.5], unit2);
fg.push_back(f4);

end