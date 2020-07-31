from .gtsam import *

def _init():
    import numpy as np

    global Point2  # export function

    def Point2(x=0, y=0):
        return np.array([x, y], dtype=float)

    global Point3  # export function

    def Point3(x=0, y=0, z=0):
        return np.array([x, y, z], dtype=float)

    # for interactive debugging
    if __name__ == "__main__":
        # we want all definitions accessible
        globals().update(locals())


_init()
