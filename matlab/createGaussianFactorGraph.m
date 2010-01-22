% create a linear factor graph
% The non-linear graph above evaluated at NoisyConfig
function fg = createGaussianFactorGraph()

c = createNoisyConfig(); 

% Create
fg = GaussianFactorGraph;

% Create shared Noise models
sigma0_1 = SharedDiagonal([0.1;0.1]);
sigma0_2 = SharedDiagonal([0.2;0.2]);

% prior on x1
A11=eye(2);
b = - c.get('x1');
f1 = GaussianFactor('x1', A11, b, sigma0_1); % generate a Gaussian factor of odometry
fg.push_back(f1);

% odometry between x1 and x2
A21=-eye(2);
A22=eye(2);
b = [.2;-.1];
f2 = GaussianFactor('x1', A21,  'x2', A22, b,sigma0_1);
fg.push_back(f2);

% measurement between x1 and l1
A31=-eye(2);
A33=eye(2);
b = [0;.2];
f3 = GaussianFactor('x1', A31, 'l1', A33, b,sigma0_2);
fg.push_back(f3);

% measurement between x2 and l1
A42=-eye(2);
A43=eye(2);
b = [-.2;.3];
f4 = GaussianFactor('x2', A42, 'l1', A43, b,sigma0_2);
fg.push_back(f4);

end