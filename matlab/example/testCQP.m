% Test example for simple Constrained Quadratic Programming problem

A = [-1    -1;
     -2     1;
      1    -1];
At = A';
B = 2*eye(3,3);

b = [4; -2];
g = zeros(3,1);

[delta lambda] = solveCQP(B, A, At, g, b);

expX = [2/7 10/7 -6/7]';

state_error = norm(expX-delta)