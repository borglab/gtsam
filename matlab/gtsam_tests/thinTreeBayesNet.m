function [bayesNet, tree] = thinTreeBayesNet(d,w)
import gtsam.*
bayesNet = GaussianBayesNet;
tree = thinTree(3,2);

% Filling the tree

% Creation of the root
gc = gtsam.GaussianConditional(1, 5*rand(1), 5*rand(1), 3*rand(1));
% Getting it into the GaussianBayesNet
bayesNet.push_front(gc);

for i=1:2^tree.getNumberOfElements()
    % Getting the parents of that node
    parents = tree.getParents(i);
    % Create and link the corresponding GaussianConditionals
    if tree.getW == 1
        % Creation of the GaussianConditional
        gc = gtsam.GaussianConditional(parents(1), 5*rand(1), 5*rand(1));
        % Getting it into the GaussianBayesNet
        bayesNet.push_front(gc);
        % Getting it in the thinTree
        t = tree.addContent({gc,parents}, i);
    elseif tree.getW == 2
        % Creation of the GaussianConditional
        gc = gtsam.GaussianConditional(parents(2), 5*rand(1), 5*rand(1), parents(1), 5*rand(1), 5*rand(1));
        % Getting it into the GaussianBayesNet
        bayesNet.push_front(gc);
        % Getting it in the thinTree
        t = tree.addContent({gc,parents}, i);
    end
end
end