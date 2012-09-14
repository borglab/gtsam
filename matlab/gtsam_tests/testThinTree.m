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
%  * @file testThinTree.cpp
%  * @brief Test of binary tree
%  * @date Sep 13, 2012
%  * @author Frank Dellaert
%  * @author Jean-Guillaume Durand
%  */

%% Clear working space
clc, close all, clear all;

%% Create different trees for our example
import gtsam.*
t0 = thinTree(2,1);
t1 = thinTree(3,2);
% Add contents in it
% TODO
%% Create the set of expected output TestValues
expectedNumberOfNodes0 = 3;
expectedNumberOfNodes1 = 7;
expectedParentsOf6in1 = [3 1];
expectedParentsOf7in1 = [3 1];

%% Run the tests
% Tree depth
%TODO
% Number of parents for each node
%TODO
% Number of elements
EQUALITY('expectedNumberOfNodes0,t0.getNumberOfElements', expectedNumberOfNodes0,t0.getNumberOfElements);
EQUALITY('expectedNumberOfNodes1,t1.getNumberOfElements', expectedNumberOfNodes1,t1.getNumberOfElements);
% Parents linking
EQUALITY('expectedParentsOf6in1,t1.getParents(6)', expectedParentsOf6in1,t1.getParents(6));
EQUALITY('expectedParentsOf7in1,t1.getParents(7)', expectedParentsOf7in1,t1.getParents(7));
% Adding an element

bn = thinTreeBayesNet(3,2);
EQUALITY('7 = bn.size', 7, bn.size);