% create a linear factor graph
% The non-linear graph above evaluated at NoisyConfig
function fg = createLinearFactorGraph()

c = createNoisyConfig();

% Create
fg = LinearFactorGraph;

% prior on x1
A11=[10 0; 0 10];
b = - c.get('x1')/0.1;

f1 = LinearFactor('x1', A11, b);
fg.push_back(f1);

% odometry between x1 and x2
A21=[-10 0; 0 -10];
A22=[ 10 0; 0  10];
b = [2;-1];

f2 = LinearFactor('x1', A21,  'x2', A22, b);
fg.push_back(f2);

% measurement between x1 and l1
A31=[-5 0; 0 -5];
A32=[ 5 0; 0  5];
b = [0;1];

f3 = LinearFactor('x1', A31, 'l1', A32, b);
fg.push_back(f3);

% measurement between x2 and l1
A41=[-5 0; 0 -5];
A42=[ 5 0; 0  5];
b = [-1;1.5];

f4 = LinearFactor('x2', A41, 'l1', A42, b);
fg.push_back(f4);

end