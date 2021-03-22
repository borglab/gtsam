function bayesTree = thinBayesTree(depth, width)
    import gtsam.*
    bayesNet = thinTreeBayesNet(depth, width);
    fg = GaussianFactorGraph(bayesNet);
    bayesTree = fg.eliminateMultifrontal();
end