%CCOLAMD_DEMO demo for ccolamd and csymamd
% minimum degree ordering algorithm.
%
% Example:
%   ccolamd_demo
%
% See also ccolamd

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.

%-------------------------------------------------------------------------------
% Print the introduction, the help info, and compile the mexFunctions
%-------------------------------------------------------------------------------

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'ccolamd/csymamd demo.') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help ccolamd_demo ;

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'ccolamd help information:') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help ccolamd ;

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'csymamd help information:') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help csymamd ;

%-------------------------------------------------------------------------------
% Solving Ax=b
%-------------------------------------------------------------------------------

n = 100 ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Solving Ax=b for a small %d-by-%d random matrix:', n, n) ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, '\nNote: Random sparse matrices are AWFUL test cases.\n') ;
fprintf (1, 'They''re just easy to generate in a demo.\n') ;

% set up the system

rand ('state', 0) ;
randn ('state', 0) ;
spparms ('default') ;
A = sprandn (n, n, 2/n) + speye (n) ;
b = (1:n)' ;

clf ;
subplot (3,4,1)
spy (A)
title ('original matrix')

fprintf (1, '\n\nSolving via lu (PAQ = LU), where Q is from ccolamd:\n') ;
q = ccolamd (A, 1) ;
I = speye (n) ;
Q = I (:, q) ;
[L,U,P] = lu (A*Q) ;
fl = luflops (L, U) ;
x = Q * (U \ (L \ (P * b))) ;
fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
fprintf (1, 'residual:                                     %e\n', norm (A*x-b));
subplot (3,4,2) ;
spy (L|U) ;
title ('LU with ccolamd') ;

try
fprintf (1, '\n\nSolving via lu (PAQ = LU), where Q is from colamd:\n') ;
q = colamd (A) ;
I = speye (n) ;
Q = I (:, q) ;
[L,U,P] = lu (A*Q) ;
fl = luflops (L, U) ;
x = Q * (U \ (L \ (P * b))) ;
fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
fprintf (1, 'residual:                                     %e\n', norm (A*x-b));
subplot (3,4,3) ;
spy (L|U) ;
title ('LU with colamd') ;
catch
fprintf (1, 'You have a very old version of MATLAB (no colamd) \n') ;
end

fprintf (1, '\n\nSolving via lu (PA = LU), without regard for sparsity:\n') ;
[L,U,P] = lu (A) ;
fl = luflops (L, U) ;
x = U \ (L \ (P * b)) ;
fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
fprintf (1, 'residual:                                     %e\n', norm (A*x-b));
subplot (3,4,4) ;
spy (L|U) ;
title ('LU with no ordering') ;

%-------------------------------------------------------------------------------
% Large demo for ccolamd
%-------------------------------------------------------------------------------

% Since the analysis will be done on the Cholesky factorization of A'A,
% set the knob to tell ccolamd to order for Cholesky, not LU.

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Large demo for ccolamd (symbolic analysis only):') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;

rand ('state', 0) ;
randn ('state', 0) ;
spparms ('default') ;
n = 1000 ;
fprintf (1, 'Generating a random %d-by-%d sparse matrix.\n', n, n) ;
A = sprandn (n, n, 2/n) + speye (n) ;

subplot (3,4,5)
spy (A)
title ('original matrix')

fprintf (1, '\n\nUnordered matrix:\n') ;
[lnz,h,parent,post,R] = symbfact (A, 'col') ;
fprintf (1, 'nz in Cholesky factors of A''A:            %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A''A:           %d\n', sum (lnz.^2)) ;
subplot (3,4,6) ;
spy (R) ;
title ('Cholesky with no ordering') ;

tic ;
p = ccolamd (A) ;
t = toc ;
[lnz,h,parent,post,R] = symbfact (A (:,p), 'col') ;
fprintf (1, '\n\nccolamd run time:                         %f\n', t) ;
fprintf (1, 'ccolamd ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(:,p)''A(:,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(:,p)''A(:,p): %d\n', sum (lnz.^2)) ;
subplot (3,4,7) ;
spy (R) ;
title ('Cholesky with ccolamd') ;

try
tic ;
p = colamd (A) ;
t = toc ;
[lnz,h,parent,post,R] = symbfact (A (:,p), 'col') ;
fprintf (1, '\n\ncolamd run time:                          %f\n', t) ;
fprintf (1, 'colamd ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(:,p)''A(:,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(:,p)''A(:,p): %d\n', sum (lnz.^2)) ;
subplot (3,4,8) ;
spy (R) ;
title ('Cholesky with colamd') ;
catch
fprintf (1, 'You have a very old version of MATLAB (no colamd) \n') ;
end

%-------------------------------------------------------------------------------
% Large demo for csymamd
%-------------------------------------------------------------------------------

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Large demo for csymamd (symbolic analysis only):') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;

fprintf (1, 'Generating a random symmetric %d-by-%d sparse matrix.\n', n, n) ;
A = A+A' ;

subplot (3,4,9) ;
spy (A)
title ('original matrix')

fprintf (1, '\n\nUnordered matrix:\n') ;
[lnz,h,parent,post,R] = symbfact (A, 'sym') ;
fprintf (1, 'nz in Cholesky factors of A:       %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A:      %d\n', sum (lnz.^2)) ;
subplot (3,4,10) ;
spy (R) ;
title ('Cholesky with no ordering') ;

tic ;
p = csymamd (A) ;
t = toc ;
[lnz,h,parent,post,R] = symbfact (A (p,p), 'sym') ;
fprintf (1, '\n\ncsymamd run time:                  %f\n', t) ;
fprintf (1, 'csymamd ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(p,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(p,p): %d\n', sum (lnz.^2)) ;
subplot (3,4,11) ;
spy (R) ;
title ('Cholesky with csymamd') ;

try
tic ;
p = symamd (A) ;
t = toc ;
lnz = symbfact (A (p,p), 'sym') ;
fprintf (1, '\n\nsymamd run time:                   %f\n', t) ;
fprintf (1, 'symamd ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(p,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(p,p): %d\n', sum (lnz.^2)) ;
subplot (3,4,12) ;
spy (R) ;
title ('Cholesky with symamd') ;
catch
fprintf (1, 'You have a very old version of MATLAB (no symamd) \n') ;
end

drawnow
