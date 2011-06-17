function fl = luflops (L, U)
%LUFLOPS compute the flop count for sparse LU factorization
%
%  Example:
%      fl = luflops (L,U)
%
%  Given a sparse LU factorization (L and U), return the flop count required
%  by a conventional LU factorization algorithm to compute it.   L and U can
%  be either sparse or full matrices.  L must be lower triangular and U must
%  be upper triangular.  Do not attempt to use this on the permuted L from
%  [L,U] = lu (A).  Instead, use [L,U,P] = lu (A) or [L,U,P,Q] = lu (A).
%
%  Note that there is a subtle undercount in this estimate.  Suppose A is
%  completely dense, but during LU factorization exact cancellation occurs,
%  causing some of the entries in L and U to become identically zero.  The
%  flop count returned by this routine is an undercount.  There is a simple
%  way to fix this (L = spones (L) + spones (tril (A))), but the fix is partial.
%  It can also occur that some entry in L is a "symbolic" fill-in (zero in
%  A, but a fill-in entry and thus must be computed), but numerically
%  zero.  The only way to get a reliable LU factorization would be to do a
%  purely symbolic factorization of A.  This cannot be done with
%  symbfact (A, 'col').
%
%  See NA Digest, Vol 00, #50, Tuesday, Dec. 5, 2000
%
%  See also symbfact

% Copyright 1998-2007, Timothy A. Davis

Lnz = full (sum (spones (L))) - 1 ;	% off diagonal nz in cols of L
Unz = full (sum (spones (U')))' - 1 ;	% off diagonal nz in rows of U
fl = 2*Lnz*Unz + sum (Lnz) ;

