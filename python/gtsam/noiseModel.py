# This trick is to allow direct import of sub-modules
# without this, we can only do `from gtsam.gtsam.noiseModel import X`
# with this trick, we can do `from gtsam.noiseModel import X`
from .gtsam.noiseModel import *