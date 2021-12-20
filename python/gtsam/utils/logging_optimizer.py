"""
Optimization with logging via a hook.
Author: Jing Wu and Frank Dellaert
"""
# pylint: disable=invalid-name

from gtsam import NonlinearOptimizer, NonlinearOptimizerParams
import gtsam


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

    # Iterative loop
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
