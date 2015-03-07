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
%  * @file testThinTreeBayesNet.cpp
%  * @brief Test of binary tree
%  * @date Sep 13, 2012
%  * @author Frank Dellaert
%  * @author Jean-Guillaume Durand
%  */

%% Run the tests
import gtsam.*
[bayesNet tree] = thinTreeBayesNet(4,2);
EQUALITY('7 = bayesNet.size', 7, bayesNet.size);
gtsam.plotBayesNet(bayesNet);