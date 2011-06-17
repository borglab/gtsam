function ccolamd_test
%CCOLAMD_TEST extensive test of ccolamd and csymamd
%
% Example:
%   ccolamd_test
%
% See also csymamd, ccolamd, ccolamd_make.

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.

help ccolamd_test

global ccolamd_default_knobs csymamd_default_knobs
ccolamd_default_knobs = [0 10 10 1 0] ;
csymamd_default_knobs = [10 1 0] ;

    fprintf ('Compiling ccolamd, csymamd, and test mexFunctions.\n') ;
    ccolamd_make ;

    d = '' ;
    if (~isempty (strfind (computer, '64')))
	d = '-largeArrayDims' ;
    end
    src = '../Source/ccolamd.c ../Source/ccolamd_global.c' ;
    cmd = sprintf ('mex -DDLONG -O %s -I../../UFconfig -I../Include ', d) ;
    eval ([cmd 'ccolamdtestmex.c ' src]) ;
    eval ([cmd 'csymamdtestmex.c ' src]) ;
    fprintf ('Done compiling.\n') ; 


fprintf ('\nThe following codes will be tested:\n') ;
which ccolamd 
which csymamd
which ccolamdtestmex
which csymamdtestmex

fprintf ('\nStarting the tests.  Please be patient.\n') ;

h = waitbar (0, 'COLAMD test') ;

rand ('state', 0) ;
randn ('state', 0) ;

A = sprandn (500,500,0.4) ;

p = ccolamd (A, [0 10 10 1 1]) ; check_perm (p, A) ;
p = ccolamd (A, [1  2  7 1 1]) ; check_perm (p, A) ;
p = ccolamd (A, [1  2 10 0 1]) ; check_perm (p, A) ;
p = ccolamd (A, [9  2  3 1 1]) ; check_perm (p, A) ;

p = csymamd (A, [10 1 1]) ; check_perm (p, A) ;
p = csymamd (A, [4  1 1]) ; check_perm (p, A) ;
p = csymamd (A, [9  0 1]) ; check_perm (p, A) ;

fprintf ('Null matrices') ;
A = zeros (0,0) ;
A = sparse (A) ;

p = ccolamd (A) ;
check_perm (p, A) ;

p = csymamd (A) ;
check_perm (p, A) ;

A = zeros (0, 100) ;
A = sparse (A) ;
p = ccolamd (A) ;
check_perm (p, A) ;

A = zeros (100, 0) ;
A = sparse (A) ;
p = ccolamd (A) ;
check_perm (p, A) ;
fprintf (' OK\n') ;


fprintf ('Matrices with a few dense row/cols\n') ;
for trial = 1:20

    waitbar (trial/20, h, 'CCOLAMD: dense rows/cols') ;

    % random square unsymmetric matrix
    A = rand_matrix (1000, 1000, 1, 10, 20) ;
    [m n] = size (A) ;

    cmember = irand (min (trial,n), n) ;

    for tol = [0:.1:2 3:20 1e6]
	B = A + A' ;

	p = ccolamd (A, [ ]) ;		  check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 1]) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1]) ;  check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 0]) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1]) ;  check_perm (p, A) ;
	p = csymamd (A, [tol 1]) ;	  check_perm (p, A) ;
	p = csymamd (A, tol) ;		  check_perm (p, A) ;
	p = csymamd (A, [ ]) ;		  check_perm (p, A) ;
	p = csymamd (B, [tol 0]) ;	  check_perm (p, A) ;
	p = ccolamd (A, [0 tol -1 1]) ;   check_perm (p, A) ;
	p = ccolamd (A, [0 -1 tol 1]) ;   check_perm (p, A) ;

	% check with non-null cmember

	p = ccolamd (A, [ ], cmember) ;		   check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 1], cmember) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1], cmember) ;  check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 0], cmember) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1], cmember) ;  check_perm (p, A) ;
	p = csymamd (A, [tol 1], cmember) ;	   check_perm (p, A) ;
	p = csymamd (A, tol, cmember) ;		   check_perm (p, A) ;
	p = csymamd (A, [ ], cmember) ;		   check_perm (p, A) ;
	p = csymamd (B, [tol 0], cmember) ;	   check_perm (p, A) ;
	p = ccolamd (A, [0 tol -1 1], cmember) ;   check_perm (p, A) ;
	p = ccolamd (A, [0 -1 tol 1], cmember) ;   check_perm (p, A) ;

	p = ccolamd (A, [ ], [ ]) ;	       check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 1], [ ]) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1], [ ]) ;  check_perm (p, A) ;
	p = ccolamd (A, [1 tol tol 0], [ ]) ;  check_perm (p, A) ;
	p = ccolamd (A, [0 tol tol 1], [ ]) ;  check_perm (p, A) ;
	p = csymamd (A, [tol 1], [ ]) ;	       check_perm (p, A) ;
	p = csymamd (A, tol, [ ]) ;	       check_perm (p, A) ;
	p = csymamd (A, [ ], [ ]) ;	       check_perm (p, A) ;
	p = csymamd (B, [tol 0], [ ]) ;	       check_perm (p, A) ;
	p = ccolamd (A, [0 tol -1 1], [ ]) ;   check_perm (p, A) ;
	p = ccolamd (A, [0 -1 tol 1], [ ]) ;   check_perm (p, A) ;

    end
end
fprintf (' OK\n') ;

fprintf ('General matrices\n') ;
for trial = 1:400

    waitbar (trial/400, h, 'CCOLAMD: with dense rows/cols') ;

    % matrix of random mtype
    mtype = irand (3) ;
    A = rand_matrix (2000, 2000, mtype, 0, 0) ;
    p = ccolamd (A) ;
    check_perm (p, A) ;

    if (mtype == 3)
	p = csymamd (A) ;
	check_perm (p, A) ;
    end

end
fprintf (' OK\n') ;



fprintf ('Test error handling with invalid inputs\n') ;

% Check different erroneous input.
for trial = 1:30

    waitbar (trial/30, h, 'CCOLAMD: error handling') ;

    A = rand_matrix (1000, 1000, 2, 0, 0) ;

    for err = 1:13

        p = Tcolamd (A, [ccolamd_default_knobs 1 err], [ ]) ;
        if (p(1) ~= -1)							    %#ok
	    check_perm (p, A) ;
	end

	if (err == 1)
	    % check different (valid) input args to ccolamd
	    p = Acolamd (A) ;
	    p2 = Acolamd (A, [ccolamd_default_knobs 0 0]) ;
	    if (any (p ~= p2))
		error ('ccolamd: mismatch 1!') ;
	    end
	end

	B = A'*A ;
        p = Tsymamd (B, [-1 1 0 err], [ ]) ;
        if (p(1) ~= -1)							    %#ok
	    check_perm (p, A) ;
	end

	if (err == 1)

	    % check different (valid) input args to csymamd
	    p = Asymamd (B) ;
	    check_perm (p, A) ;
	    p2 = Asymamd (B, [csymamd_default_knobs 0]) ;
	    if (any (p ~= p2))
		error ('symamd: mismatch 1!') ;
	    end
	end

    end

end
fprintf (' OK\n') ;

fprintf ('Matrices with a few empty columns\n') ;

for trial = 1:400

    waitbar (trial/400, h, 'CCOLAMD: with empty rows/cols') ;

    % some are square, some are rectangular
    n = 0 ;
    while (n < 5)
	A = rand_matrix (1000, 1000, irand (2), 0, 0) ;
	[m n] = size (A) ;
    end

    % Add 5 null columns at random locations.
    null_col = randperm (n) ;
    A (:, null_col) = 0 ;

    % Order the matrix and make sure that the null columns are ordered last.
    p = ccolamd (A, [1 1e6 1e6 0]) ;
    check_perm (p, A) ;

    % find all null columns in A
    null_col = find (sum (spones (A), 1) == 0) ;
    nnull = length (null_col) ;
    if (any (null_col ~= p ((n-nnull+1):n)))
	error ('ccolamd: Null cols are not ordered last in natural order') ;
    end

end
fprintf (' OK\n') ;

fprintf ('Matrices with a few empty rows and columns\n') ;

for trial = 1:400

    waitbar (trial/400, h, 'CCOLAMD: with empty rows/cols') ;

    % symmetric matrices
    n = 0 ;
    while (n < 5)
	A = rand_matrix (1000, 1000, 3, 0, 0) ;
	[m n] = size (A) ;
    end

    % Add 5 null columns and rows at random locations.
    null_col = randperm (n) ;
    A (:, null_col) = 0 ;
    A (null_col, :) = 0 ;

    % Order the matrix and make sure that the null rows/cols are ordered last.
    p = csymamd (A, -1) ;
    check_perm (p, A) ;

    % find all null rows/columns in A
    Alo = tril (A, -1) ;
    null_col = ...
	find ((sum (spones (Alo), 1) == 0) & (sum (spones (Alo), 2) == 0)') ;
    nnull = length (null_col) ;
    if (any (null_col ~= p ((n-nnull+1):n)))
	error ('csymamd: Null cols are not ordered last in natural order') ;
    end

end
fprintf (' OK\n') ;

fprintf ('Matrices with a few empty rows\n') ;

% Test matrices with null rows inserted.

for trial = 1:400

    waitbar (trial/400, h, 'CCOLAMD: with null rows') ;
    m = 0 ;
    while (m < 5)
	A = rand_matrix (1000, 1000, 2, 0, 0) ;
	m = size (A,1) ;
    end

    % Add 5 null rows at random locations.
    null_row = randperm (m) ;
    null_row = sort (null_row (1:5)) ;
    A (null_row, :) = 0 ;

    p = ccolamd (A) ;
    check_perm (p, A) ;

end
fprintf (' OK\n') ;

fprintf ('\nccolamd and csymamd:  all tests passed\n\n') ;
close (h) ;

%-------------------------------------------------------------------------------

function p = Acolamd (S, knobs)
% Acolamd:  compare ccolamd and Tcolamd results

global ccolamd_default_knobs

if (nargin < 2)
    p = ccolamd (S) ;
    p1 = Tcolamd (S, [ccolamd_default_knobs 0 0], [ ]) ;
else
    p = ccolamd (S, knobs) ;
    p1 = Tcolamd (S, knobs, [ ]) ;
end

check_perm (p, S) ;
check_perm (p1, S) ;

if (any (p1 ~= p))
    narg = nargin ;
    if (nargin == 2)
	save bad S narg knobs
    else
	save bad S narg
    end
    error ('Acolamd mismatch!') ;
end

%-------------------------------------------------------------------------------

function p = Asymamd (S, knobs)
% Asymamd:  compare csymamd and Tsymamd results

global csymamd_default_knobs

if (nargin < 2)
    p = csymamd (S) ;
    p1 = Tsymamd (S, [csymamd_default_knobs 0], [ ]) ;
else
    p = csymamd (S, knobs) ;
    p1 = Tsymamd (S, knobs, [ ]) ;
end

if (any (p1 ~= p))
    error ('Asymamd mismatch!') ;
end


%-------------------------------------------------------------------------------

function check_perm (p, A, cmember)
% check_perm:  check for a valid permutation vector

if (isempty (A) & isempty (p))						    %#ok
    % empty permutation vectors of empty matrices are OK
    return
end

if (isempty (p))
    error ('Bad permutation: cannot be empty') ;
end

[m n] = size (A) ;
[p_m p_n] = size (p) ;
if (p_n == 1)
    % force p to be a row vector
    p = p' ;
    [p_m p_n] = size (p) ;
end

if (n ~= p_n)
    error ('Bad permutation: wrong size') ;
end

if (p_m ~= 1) ;
    % p must be a vector
    error ('Bad permutation: not a vector') ;
else
    if (any (sort (p) - (1:p_n)))
	error ('Bad permutation') ;
    end
end

if (nargin > 2)
    % check cmember
    c = cmember (p) ;
    % c must be monotonically non-decreasing
    c = diff (c) ;
    if (any (c < 0))
	error ('permutation breaks the cmember constraints') ;
    end
end

%-------------------------------------------------------------------------------

function i = irand (n,s)
% irand: return a random vector of size s, with values between 1 and n
if (nargin == 1)
    s = 1 ;
end
i = min (n, 1 + floor (rand (1,s) * n)) ;

%-------------------------------------------------------------------------------

function A = rand_matrix (n_max, m_max, mtype, d_rows, d_cols)
% rand_matrix:  return a random sparse matrix
%
% A = rand_matrix (n_max, m_max, mtype, d_rows, d_cols)
%
% A binary matrix of random size, at most n_max-by-m_max, with d_rows dense rows
% and d_cols dense columns.
%
% mtype 1: square unsymmetric (m_max is ignored)
% mtype 2: rectangular
% mtype 3: symmetric (m_max is ignored)

n = irand (n_max) ;
if (mtype ~= 2)
    % square
    m = n ;
else
    m = irand (m_max) ;
end

A = sprand (m, n, 10 / max (m,n)) ;

if (d_rows > 0)
    % add dense rows
    for k = 1:d_rows
	i = irand (m) ;
	nz = irand (n) ;
	p = randperm (n) ;
	p = p (1:nz) ;
	A (i,p) = 1 ;
    end
end

if (d_cols > 0)
    % add dense cols
    for k = 1:d_cols
	j = irand (n) ;
	nz = irand (m) ;
	p = randperm (m) ;
	p = p (1:nz) ;
	A (p,j) = 1 ;
    end
end

A = spones (A) ;

% ensure that there are no empty columns
d = find (full (sum (A,1)) == 0) ;			    %#ok
A (m,d) = 1 ;						    %#ok

% ensure that there are no empty rows
d = find (full (sum (A,2)) == 0) ;			    %#ok
A (d,n) = 1 ;						    %#ok

if (mtype == 3)
    % symmetric
    A = A + A' + speye (n) ;
end

A = spones (A) ;

%-------------------------------------------------------------------------------
% Tcolamd:  run ccolamd in a testing mode
%-------------------------------------------------------------------------------

function p = Tcolamd (S, knobs, cmember)

% knobs (5) = 1 ;
p = ccolamdtestmex (S, knobs, cmember) ;

if (p (1) ~= -1)
    check_perm (p, S) ;
end


%-------------------------------------------------------------------------------
% Tsymamd: run csymamd in a testing mode
%-------------------------------------------------------------------------------

function p = Tsymamd (S, knobs, cmember)

% knobs (2) = 1 ;
p = csymamdtestmex (S, knobs, cmember) ;

if (p (1) ~= -1)
    check_perm (p, S) ;
end

