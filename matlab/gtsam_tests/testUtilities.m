%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Checks for results of functions in utilities namespace
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Create keys for variables
x1 = symbol('x',1); x2 = symbol('x',2); x3 = symbol('x',3);

actual = utilities.createKeyList([1;2;3]);
CHECK('KeyList', isa(actual,'gtsam.KeyList'));
CHECK('size==3', actual.size==3);
CHECK('actual.front==1', actual.front==1);

actual = utilities.createKeyList('x',[1;2;3]);
CHECK('KeyList', isa(actual,'gtsam.KeyList'));
CHECK('size==3', actual.size==3);
CHECK('actual.front==x1', actual.front==x1);

actual = utilities.createKeyVector([1;2;3]);
CHECK('KeyVector', isa(actual,'gtsam.KeyVector'));
CHECK('size==3', actual.size==3);
CHECK('actual.at(0)==1', actual.at(0)==1);

actual = utilities.createKeyVector('x',[1;2;3]);
CHECK('KeyVector', isa(actual,'gtsam.KeyVector'));
CHECK('size==3', actual.size==3);
CHECK('actual.at(0)==x1', actual.at(0)==x1);

actual = utilities.createKeySet([1;2;3]);
CHECK('KeySet', isa(actual,'gtsam.KeySet'));
CHECK('size==3', actual.size==3);
CHECK('actual.count(1)', actual.count(1));

actual = utilities.createKeySet('x',[1;2;3]);
CHECK('KeySet', isa(actual,'gtsam.KeySet'));
CHECK('size==3', actual.size==3);
CHECK('actual.count(x1)', actual.count(x1));

