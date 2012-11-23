
badscale = 1e-14;

% Create random Jacobian
A = [
    badscale.*(rand(3)-.5) zeros(3)   zeros(3,6)
    zeros(3) (rand(3)-.5)             zeros(3,6)
    badscale.*(rand(2,6)-.5)          zeros(2,6)
    zeros(2,6)                        badscale.*(rand(2,6)-.5)
    eye(6)                            -eye(6)
    ];
b = [
    repmat(badscale, 3,1)
    repmat(1, 3,1)
    repmat(badscale, 2,1)
    repmat(badscale, 2,1)
    repmat(1, 6,1)
    ];

% Solve with MATLAB
x_true = A \ b;

% Form Hessian
L = A'*A;
l = A'*b;

% Eliminate badly-conditioned system.  Use this custom choleskyNaive function
% that does no internal pivoting or scaling to simulate the case where the
% bad conditioning occurs between fronts.
R = choleskyNaive(L);
if isempty(R)
    fprintf('Cholesky failed on badly-scaled system with a negative pivot\n');
else
    clear opts
    opts.LT = true;
    d = linsolve(R', l, opts);

    % Solve badly-conditioned system.
    clear opts
    opts.UT = true;
    x = linsolve(R, d, opts);
    
    % Compute error
    fprintf('Solution error with Cholesky on badly-scaled system: %g\n', ...
        max(abs(x - x_true)));
end

% Form scaled system from Jacobian
%Si = full(qr(sparse(A)));
%Si = diag(sqrt(diag(L)));
S = [
   eye(6) zeros(6)
   eye(6) eye(6) ];
Ap = A * S;
Lp = Ap'*Ap;
lp = Ap'*b;

% Eliminate scaled system
Rp = choleskyNaive(Lp);
if isempty(Rp)
    fprintf('Cholesky failed on scaled system with a negative pivot\n');
else
    clear opts
    opts.LT = true;
    dp = linsolve(Rp', lp, opts);

    % Solve rescaled system.
    clear opts
    opts.UT = true;
    xp = S * linsolve(Rp, dp, opts);
    
    % Compute error
    fprintf('Solution error with Cholesky from rescaled Jacobian: %g\n', ...
        max(abs(xp - x_true)));
end


% Form scaled system from Hessian
Lph = S' * (L * S);
lph = S' * l;

% Eliminate scaled system
Rph = choleskyNaive(Lph);
if isempty(Rph)
    fprintf('Cholesky failed on scaled system with a negative pivot\n');
else
    clear opts
    opts.LT = true;
    dph = linsolve(Rph', lph, opts);

    % Solve rescaled system.
    clear opts
    opts.UT = true;
    xph = S * linsolve(Rph, dph, opts);
    
    % Compute error
    fprintf('Solution error with Cholesky from rescaled Hessian: %g\n', ...
        max(abs(xph - x_true)));
end

