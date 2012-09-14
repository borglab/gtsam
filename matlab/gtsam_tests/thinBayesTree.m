function bayesTree = thinBayesTree(depth, width)
    import gtsam.*
    bayesNet = thinTreeBayesNet(depth, width);
    bayesTree = GaussianBayesTree(bayesNet);
end