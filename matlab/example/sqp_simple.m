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
lam0 = 0.5;
x = x0; lam = lam0;

maxIt = 1;

for i=1:maxIt
    
    x1 = x(1); x2 = x(2);

    % evaluate normal functions
    fx = (x2-2)^2 - x1^2;
    cx = 4*x1^2 + x2^2 - 1;

    % evaluate derivatives in x
    dfx = [-2*x1; 2*(x2-2)];
    dcx = [8*x1; 2*x2]; 
    
    % evaluate hessians in x
    ddfx = diag([-2, 2]);
    ddcx = diag([8, 2]);

    % construct and solve CQP subproblem
    Bgn = dfx * dfx' - lam * dcx * dcx' % GN approx
    Ba = ddfx - lam * ddcx % analytic hessians
    B = Ba;
    g = dfx;
    h = -cx;
    [delta lambda] = solveCQP(B, -dcx, -dcx', g, h);

    % update 
    x = x + delta
    lam = lambda

end

% verify
xstar = [0; 1];
lamstar = -1;
display(fx)
display(cx)
final_error = norm(x-xstar) + norm(lam-lamstar)