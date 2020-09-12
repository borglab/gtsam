# This trick is to allow direct import of sub-modules
# without this, we can only do `from gtsam.gtsam.symbol_shorthand import X`
# with this trick, we can do `from gtsam.symbol_shorthand import X`
from .gtsam.symbol_shorthand import *