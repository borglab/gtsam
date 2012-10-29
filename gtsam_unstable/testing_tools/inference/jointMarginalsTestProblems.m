% This script tests the math of correcting joint marginals with root
% shortcuts, using only simple matrix math and avoiding any GTSAM calls.
% It turns out that it is not possible to correct root shortcuts for
% joinoint marginals, thus actual_ij_1_2__ will not equal
% expected_ij_1_2__.

%% Check that we get the BayesTree structure we are expecting
fg = gtsam.SymbolicFactorGraph;
fg.push_factor(1,3,7,10);
fg.push_factor(2,5,7,10);
fg.push_factor(3,4,7,8);
fg.push_factor(5,6,7,8);
fg.push_factor(7,8,9,10);
fg.push_factor(10,11);
fg.push_factor(11);
fg.push_factor(7,8,9);
fg.push_factor(7,8);
fg.push_factor(5,6);
fg.push_factor(3,4);

bt = gtsam.SymbolicMultifrontalSolver(fg).eliminate;
bt

%%
%  P( 10 11)
%    P( 7 8 9 | 10)
%      P( 5 6 | 7 8 10)
%        P( 2 | 5 7 10)
%      P( 3 4 | 7 8 10)
%        P( 1 | 3 7 10)
A = [ ...
  % 1  2  3  4  5  6  7  8  9 10 11  b
    1  0  2  0  0  0  3  0  0  4  0  5
    0  6  0  0  7  0  8  0  0  9  0 10
    0  0 11 12  0  0 13 14  0 15  0 16
    0  0  0  0 17 18 19 20  0 21  0 22
    0  0  0  0  0  0 23 24 25 26  0 27
    0  0  0  0  0  0  0  0  0 28 29 30
    0  0  0  0  0  0  0  0  0  0 31 32
    0  0  0  0  0  0 33 34 35  0  0 36
    0  0  0  0  0  0 37 38  0  0  0 39
    0  0  0  0 40 41  0  0  0  0  0 42
    0  0 43 44  0  0  0  0  0  0  0 45
    %0  0  0  0 46  0  0  0  0  0  0 47
    %0  0 48  0  0  0  0  0  0  0  0 49
    ];

% Compute shortcuts
shortcut3_7__10_11 = shortcut(A, [3,7], [10,11]);
shortcut5_7__10_11 = shortcut(A, [5,7], [10,11]);

% Factor into pieces
% Factor p(5,7|10,11) into p(5|7,10,11) p(7|10,11)
[ si_si__sij_R, si_sij__R ] = eliminate(shortcut3_7__10_11, [3]);
[ sj_sj__sij_R, sj_sij__R ] = eliminate(shortcut5_7__10_11, [5]);

% Get root marginal
R__ = A([6,7], :);

% Get clique conditionals
i_1__3_7_10 = A(1, :);
j_2__5_7_10 = A(2, :);

% Check
expected_ij_1_2__ = shortcut(A, [1,2], [])

% Compute joint
ij_1_2_3_5_7_10_11 = [
    i_1__3_7_10
    j_2__5_7_10
    si_si__sij_R
    sj_sj__sij_R
    si_sij__R
    R__
    ];
actual_ij_1_2__ = shortcut(ij_1_2_3_5_7_10_11, [1,2], [])
