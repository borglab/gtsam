%-----------------------------------------------------------------------
% frank01.m: try conjugate gradient on our example graph
%-----------------------------------------------------------------------

% get matrix form H and z
fg = createGaussianFactorGraph();
ord = Ordering;
ord.push_back('x1');
ord.push_back('x2');
ord.push_back('l1');

[H,z] = fg.matrix(ord);

% form system of normal equations
A=H'*H
b=H'*z

% k=0
x = zeros(6,1)
g = A*x-b
d = -g

for k=1:5
    alpha = - (d'*g)/(d'*A*d)
    x = x + alpha*d
    g = A*x-b
    beta = (d'*A*g)/(d'*A*d)
    d = -g + beta*d
end

% Do gradient descent
% fg2 = createGaussianFactorGraph();
% zero = createZeroDelta();
% actual = fg2.gradientDescent(zero);
% CHECK(assert_equal(expected,actual,1e-2));

% Do conjugate gradient descent
% actual2 = fg2.conjugateGradientDescent(zero);
% CHECK(assert_equal(expected,actual2,1e-2));
