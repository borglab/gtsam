function [delta lambda] = solveCQP(B, A, At, g, h)

n = size(B,1);
p = size(A,2);

% form the KKT matrix system
G = [B A; At zeros(p,p)];
rhs = -[g; h];

% solve with LDL
[L D] = ldl(G);
approx_error = norm(G - L*D*L'); %% verify error
sol = L'\(D\(L\rhs));

delta = sol(1:n);
lambda = sol(n+1:size(sol));