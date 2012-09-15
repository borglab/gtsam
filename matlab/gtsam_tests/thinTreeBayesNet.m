function [bayesNet, tree] = thinTreeBayesNet(depth,width)
% thinTreeBayesNet

import gtsam.*
bayesNet = GaussianBayesNet;
tree = thinTree(depth,width);

% Add root to the Bayes net
gc = gtsam.GaussianConditional(1, 5*rand(1), 5*rand(1), 3*rand(1));
bayesNet.push_front(gc);

n=tree.getNumberOfElements();
for i=2:n
  % Getting the parents of that node
  parents = tree.getParents(i);
  di = tree.getNodeDepth(i);
  % Create and link the corresponding GaussianConditionals
  if tree.getW == 1 || di == 2
    % Creation of single-parent GaussianConditional
    gc = gtsam.GaussianConditional(n-i, 5*rand(1), 5*rand(1), n-parents(1), 5*rand(1), 5*rand(1));
  elseif tree.getW == 2 || di == 3
    % GaussianConditionalj associated with the second parent
    gc = gtsam.GaussianConditional(n-i, 5*rand(1), 5*rand(1), n-parents(1), 5*rand(1), n-parents(2), 5*rand(1), 5*rand(1));
  end
  % Add conditional to the Bayes net
  bayesNet.push_front(gc);
end

end