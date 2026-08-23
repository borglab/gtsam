% /* ----------------------------------------------------------------------------
%
%  * GTSAM Copyright 2010, Georgia Tech Research Corporation,
%  * Atlanta, Georgia 30332-0415
%  * All Rights Reserved
%  * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
%  * See LICENSE for the license information
%
%  * -------------------------------------------------------------------------- */
%
% /**
%  * @file testThinBayesTree.cpp
%  * @brief Test of binary tree
%  * @date Sep 14, 2012
%  * @author Frank Dellaert
%  * @author Jean-Guillaume Durand
%  */

%% Run the tests
bayesTree = thinBayesTree(3,2);
gtsam.EQUALITY('7 = bayesTree.size', 4, bayesTree.size);
