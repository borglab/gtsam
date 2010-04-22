% Script to perform SQP on a simple example from the SQP tutorial
% 
% Problem:
%  min(x) f(x) = (x2-2)^2 - x1^2
%    s.t. c(x) = 4x1^2 + x2^2 - 1 =0
% state is x = [x1 x2]' , with dim(state) = 2
% constraint has dim p = 1

n = 2;
p = 1;

% initial conditions
x0 = [2; 4];
lam0 = -0.5;
x = x0; lam = lam0;

maxIt = 100;

X = x0; Lam = lam0;
Bi = eye(2);

for i=1:maxIt
    i
    x1 = x(1); x2 = x(2);

    % evaluate functions
    fx = (x2-2)^2 + x1^2;
    cx = 4*x1^2 + x2^2 - 1;

    % evaluate derivatives in x
    dfx = [2*x1; 2*(x2-2)];
    dcx = [8*x1; 2*x2]; 
    dL = dfx - lam * dcx;

    % update the hessian (BFGS)
    if (i>1)
        Bis = Bi*s;
        y = dL - prev_dL;
        Bi = Bi + (y*y')/(y'*s) - (Bis*Bis')/(s'*Bis);
    end
    prev_dL = dL;
    
    % evaluate hessians in x
    ddfx = diag([2, 2]);
    ddcx = diag([8, 2]);

    % construct and solve CQP subproblem
    Bgn0 = dfx * dfx';
    Bgn1 = dfx * dfx' - lam * dcx * dcx'; % GN approx 1
    Bgn2 = dL * dL'; % GN approx 2
    Ba = ddfx - lam * ddcx; % analytic hessians

    B = ddfx;
    g = dfx;
    h = -cx;
    [delta lambda] = solveCQP(B, -dcx, -dcx', g, h);
    
    % update 
    s = 0.5*delta;
    x = x + s
    lam = lambda
    
    
    % save
    X = [X x];
    Lam = [Lam lam];
    
end

% verify
xstar = [0; 1];
lamstar = -1;
display(fx)
display(cx)
final_error = norm(x-xstar) + norm(lam-lamstar)

% draw
figure(1);
clf;
ezcontour('(x2-2)^2 + x1^2');
hold on;
ezplot('4*x1^2 + x2^2 - 1');
plot(X(1,:), X(2,:), 'r*-');
