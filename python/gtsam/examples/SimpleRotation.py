"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

This example will perform a relatively trivial optimization on
a single variable with a single factor.
"""

import numpy as np
import gtsam
from gtsam.symbol_shorthand import X

def main():
    """
    Step 1: Create a factor to express a unary constraint

    The "prior" in this case is the measurement from a sensor,
    with a model of the noise on the measurement.

    The "Key" created here is a label used to associate parts of the
    state (stored in "RotValues") with particular factors.  They require
    an index to allow for lookup, and should be unique.

    In general, creating a factor requires:
    - A key or set of keys labeling the variables that are acted upon
    - A measurement value
    - A measurement model with the correct dimensionality for the factor
    """
    prior = gtsam.Rot2.fromAngle(np.deg2rad(30))
    prior.print_('goal angle')
    model = gtsam.noiseModel.Isotropic.Sigma(dim=1, sigma=np.deg2rad(1))
    key = X(1)
    factor = gtsam.PriorFactorRot2(key, prior, model)

    """
    Step 2: Create a graph container and add the factor to it

    Before optimizing, all factors need to be added to a Graph container,
    which provides the necessary top-level functionality for defining a
    system of constraints.

    In this case, there is only one factor, but in a practical scenario,
    many more factors would be added.
    """
    graph = gtsam.NonlinearFactorGraph()
    graph.push_back(factor)
    graph.print_('full graph')

    """
    Step 3: Create an initial estimate

    An initial estimate of the solution for the system is necessary to
    start optimization.  This system state is the "Values" instance,
    which is similar in structure to a dictionary, in that it maps
    keys (the label created in step 1) to specific values.

    The initial estimate provided to optimization will be used as
    a linearization point for optimization, so it is important that
    all of the variables in the graph have a corresponding value in
    this structure.
    """
    initial = gtsam.Values()
    initial.insert(key, gtsam.Rot2.fromAngle(np.deg2rad(20)))
    initial.print_('initial estimate')

    """
    Step 4: Optimize

    After formulating the problem with a graph of constraints
    and an initial estimate, executing optimization is as simple
    as calling a general optimization function with the graph and
    initial estimate.  This will yield a new RotValues structure
    with the final state of the optimization.
    """
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    result.print_('final result')


if __name__ == '__main__':
    main()
