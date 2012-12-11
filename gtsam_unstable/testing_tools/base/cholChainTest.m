scale = 1e-10;
length = 10000;

A = spalloc(length, length, 2*length);
b = zeros(length, 1);

A(1,1) = 1;

for i = 2:length
    A(i, i-1) = 1e2*scale;
    A(i, i) = -scale;
end

L = A'*A;
eta = A'*b;

R = choleskyNaive(L);
clear opts
opts.LT = true;
d = R' \ eta; %linsolve(R', eta, opts);

clear opts
opts.UT = true;
x = R \ d; %linsolve(R, d, opts);

