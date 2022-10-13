# This trick is to allow direct import of sub-modules
# without this, we can only do `from gtsam.gtsam.gtsfm import X`
# with this trick, we can do `from gtsam.gtsfm import X`
from .gtsam.gtsfm import *