function [p, stats] = ccolamd (S, knobs, cmember)			    %#ok
%CCOLAMD constrained column approximate minimum degree permutation.
%    p = CCOLAMD(S) returns the column approximate minimum degree permutation
%    vector for the sparse matrix S.  For a non-symmetric matrix S, S(:,p)
%    tends to have sparser LU factors than S.  chol(S(:,p)'*S(:,p)) also tends
%    to be sparser than chol(S'*S).  p=ccolamd(S,1) optimizes the ordering for
%    lu(S(:,p)).  The ordering is followed by a column elimination tree post-
%    ordering.
%
%    Example:
%            p = ccolamd (S)
%            p = ccolamd (S,knobs,cmember)
%
%    knobs is an optional one- to five-element input vector, with a default
%    value of [0 10 10 1 0] if not present or empty ([ ]).  Entries not present
%    are set to their defaults.
%
%    knobs(1): if nonzero, the ordering is optimized for lu(S(:,p)).  It will
%       be a poor ordering for chol(S(:,p)'*S(:,p)).  This is the most
%       important knob for ccolamd.
%    knobs(2): if S is m-by-n, rows with more than max(16,knobs(2)*sqrt(n))
%       entries are ignored.
%    knobs(3): columns with more than max(16,knobs(3)*sqrt(min(m,n))) entries
%       are ignored and ordered last in the output permutation (subject to the
%       cmember constraints).
%    knobs(4): if nonzero, aggressive absorption is performed.
%    knobs(5): if nonzero, statistics and knobs are printed.
%
%    cmember is an optional vector of length n.  It defines the constraints on
%    the column ordering.  If cmember(j)=s, then column j is in constraint set
%    s (s must be in the range 1 to n).  In the output permutation p, all
%    columns in set 1 appear first, followed by all columns in set 2, and so
%    on.  cmember=ones(1,n) if not present or empty.  ccolamd(S,[],1:n) returns
%    1:n.
%
%    p = ccolamd(S) is about the same as p = colamd(S).  knobs and its default
%    values differ.  colamd always does aggressive absorption, and it finds an
%    ordering suitable for both lu(S(:,p)) and chol(S(:,p)'*S(:,p)); it cannot
%    optimize its ordering for lu(S(:,p)) to the extent that ccolamd(S,1) can.
%
%    See also AMD, CSYMAMD, COLAMD, SYMAMD, SYMRCM.

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.
% Supported by the National Science Foundation (DMS-9504974, DMS-9803599,
% CCR-0203270), and a grant from Sandia National Lab.

error ('ccolamd: mexFunction not found') ;
