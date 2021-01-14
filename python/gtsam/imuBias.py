# This trick is to allow direct import of sub-modules
# without this, we can only do `from gtsam.gtsam.imuBias import X`
# with this trick, we can do `from gtsam.imuBias import X`
from .gtsam.imuBias import *
