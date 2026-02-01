function [p, stats] = csymamd (S, knobs, cmember)			    %#ok
%CSYMAMD constrained symmetric approximate minimum degree permutation
%    P = CSYMAMD(S) for a symmetric positive definite matrix S, returns the
%    permutation vector p such that S(p,p) tends to have a sparser Cholesky
%    factor than S.  Sometimes CSYMAMD works well for symmetric indefinite
%    matrices too.  The matrix S is assumed to be symmetric; only the
%    strictly lower triangular part is referenced.   S must be square.  Note
%    that p=amd(S) is faster, but does not allow for a constrained ordering.
%    The ordering is followed by an elimination tree post-ordering.
%
%    See also AMD, CCOLAMD, COLAMD, SYMAMD.
%
%    Example:
%            p = csymamd (S)
%            p = csymamd (S,knobs,cmember)
%
%    knobs is an optional one- to three-element input vector, with a default
%    value of [10 1 0] if present or empty ([ ]).  Entries not present are set
%    to their defaults.
%
%    knobs(1): If S is n-by-n, then rows and columns with more than
%       max(16,knobs(1)*sqrt(n)) entries are ignored, and ordered last in the
%       output permutation (subject to the cmember constraints).
%    knobs(2): if nonzero, aggressive absorption is performed.
%    knobs(3): if nonzero, statistics and knobs are printed.
%
%    cmember is an optional vector of length n.  It defines the constraints on
%    the ordering.  If cmember(j)=s, then row/column j is in constraint set s
%    (s must be in the range 1 to n).  In the output permutation p,
%    rows/columns in set 1 appear first, followed by all rows/columns in set 2,
%    and so on.  cmember=ones(1,n) if not present or empty.  csymamd(S,[],1:n)
%    returns 1:n.
%
%    p = csymamd(S) is about the same as p = symamd(S).  knobs and its default
%    values differ.
%
%    Authors: S. Larimore, T. Davis, and S. Rajamanickam, in
%    collaboration with J. Gilbert and E. Ng.  Supported by the National
%    Science Foundation (DMS-9504974, DMS-9803599, CCR-0203270), and a grant
%    from Sandia National Lab.  See http://www.suitesparse.com
%    for ccolamd, csymamd, amd, colamd, symamd, and other related orderings.
%
%    See also AMD, CCOLAMD, COLAMD, SYMAMD, SYMRCM.

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.

error ('csymamd: mexFunction not found') ;
