"""
Optimization with logging via a hook.
Author: Jing Wu and Frank Dellaert
"""
# pylint: disable=invalid-name

from gtsam import NonlinearOptimizer, NonlinearOptimizerParams
import gtsam
from typing import Any, Callable

OPTIMIZER_PARAMS_MAP = {
    gtsam.GaussNewtonOptimizer: gtsam.GaussNewtonParams,
    gtsam.LevenbergMarquardtOptimizer: gtsam.LevenbergMarquardtParams,
    gtsam.DoglegOptimizer: gtsam.DoglegParams,
    gtsam.GncGaussNewtonOptimizer: gtsam.GaussNewtonParams,
    gtsam.GncLMOptimizer: gtsam.LevenbergMarquardtParams
}


def optimize_using(OptimizerClass, hook, *args) -> gtsam.Values:
    """ Wraps the constructor and "optimize()" call for an Optimizer together and adds an iteration
        hook.
        Example usage:
            ```python
            def hook(optimizer, error):
                print("iteration {:}, error = {:}".format(optimizer.iterations(), error))
            solution = optimize_using(gtsam.GaussNewtonOptimizer, hook, graph, init, params)
            ```
        Iteration hook's args are (optimizer, error) and return type should be None

    Args:
        OptimizerClass (T): A NonlinearOptimizer class (e.g. GaussNewtonOptimizer,
            LevenbergMarquardtOptimizer)
        hook ([T, double] -> None): Function to callback after each iteration.  Args are (optimizer,
            error) and return should be None.
        *args: Arguments that would be passed into the OptimizerClass constructor, usually:
            graph, init, [params]
    Returns:
        (gtsam.Values): A Values object representing the optimization solution.
    """
    # Add the iteration hook to the NonlinearOptimizerParams
    for arg in args:
        if isinstance(arg, gtsam.NonlinearOptimizerParams):
            arg.iterationHook = lambda iteration, error_before, error_after: hook(
                optimizer, error_after)
            break
    else:
        params = OPTIMIZER_PARAMS_MAP[OptimizerClass]()
        params.iterationHook = lambda iteration, error_before, error_after: hook(
            optimizer, error_after)
        args = (*args, params)
    # Construct Optimizer and optimize
    optimizer = OptimizerClass(*args)
    hook(optimizer, optimizer.error())  # Call hook once with init values to match behavior below
    return optimizer.optimize()


def optimize(optimizer, check_convergence, hook):
    """ Given an optimizer and a convergence check, iterate until convergence.
        After each iteration, hook(optimizer, error) is called.
        After the function, use values and errors to get the result.
        Arguments:
            optimizer (T): needs an iterate and an error function.
            check_convergence: T * float * float -> bool
            hook -- hook function to record the error
    """
    # the optimizer is created with default values which incur the error below
    current_error = optimizer.error()
    hook(optimizer, current_error)

    # Iterative loop.  Cannot use `params.iterationHook` because we don't have access to params
    # (backwards compatibility issue).
    while True:
        # Do next iteration
        optimizer.iterate()
        new_error = optimizer.error()
        hook(optimizer, new_error)
        if check_convergence(optimizer, current_error, new_error):
            return
        current_error = new_error


def gtsam_optimize(optimizer,
                   params,
                   hook):
    """ Given an optimizer and params, iterate until convergence.
        Recommend using optimize_using instead.
        After each iteration, hook(optimizer) is called.
        After the function, use values and errors to get the result.
        Arguments:
                optimizer {NonlinearOptimizer} -- Nonlinear optimizer
                params {NonlinearOptimizarParams} -- Nonlinear optimizer parameters
                hook -- hook function to record the error
    """
    def check_convergence(optimizer, current_error, new_error):
        return (optimizer.iterations() >= params.getMaxIterations()) or (
            gtsam.checkConvergence(params.getRelativeErrorTol(), params.getAbsoluteErrorTol(), params.getErrorTol(),
                                   current_error, new_error)) or (
            isinstance(optimizer, gtsam.LevenbergMarquardtOptimizer) and optimizer.lambda_() > params.getlambdaUpperBound())
    optimize(optimizer, check_convergence, hook)
    return optimizer.values()
