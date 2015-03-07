function R = shortcut(Ab, frontals, separators)
%SHORTCUT This is a simple scalar-level function for computing a "shortcut"
%of frontals conditioned on separators.  Note that the input matrix Ab is
%assumed to have a RHS vector in its right-most column, thus this rightmost
%column index should not be listed in either frontals or separators.  This
%function computes p(frontals | separators), marginalizing out all other
%variables (cooresponding to columns) in Ab.

% First marginalize out all others
cols = 1:size(Ab,2)-1;
toMarginalizeOut = setdiff(cols, [ frontals separators ]);
[ cond marg ] = eliminate(Ab, toMarginalizeOut);

% Now eliminate the frontals to get the conditional
[ cond marg ] = eliminate(marg, frontals);

R = cond;

end

