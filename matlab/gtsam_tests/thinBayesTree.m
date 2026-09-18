function bayesTree = thinBayesTree(depth, width)
    bayesNet = thinTreeBayesNet(depth, width);
    fg = gtsam.GaussianFactorGraph(bayesNet);
    bayesTree = fg.eliminateMultifrontal();
end
