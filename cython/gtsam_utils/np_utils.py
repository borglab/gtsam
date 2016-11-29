import numpy as np


def Vector(*args):
    """
    Convenient function to create numpy vector to use with gtsam cython wrapper
    Usage: Vector(1), Vector(1,2,3), Vector(3,2,4)
    """
    ret = np.asarray(args, dtype='float')
    while ret.ndim >= 2:
        ret = ret[0, :]
    return ret


def Matrix(*args):
    """
    Convenient function to create numpy matrix to use with gtsam cython wrapper
    Usage: Matrix([1]) 
           Matrix([1,2,3]) 
           Matrix((3,2,4))
           Matrix([1,2,3],[2,3,4]) 
           Matrix((1,2,3),(2,3,4))
    """
    ret = np.asarray(args, dtype='float', order='F')
    if ret.ndim == 1:
        ret = np.expand_dims(ret, axis=0)
    return ret
