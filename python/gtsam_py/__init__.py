from .gtsam import *

def Point2(x=0, y=0):
    import numpy as np
    return np.array([x, y], dtype=float)

def Point3(x=0, y=0, z=0):
    import numpy as np
    return np.array([x, y, z], dtype=float)
