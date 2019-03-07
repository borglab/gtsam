from .gtsam import *

import gtsam_unstable


def deprecated_wrapper(item, name):
    def wrapper(*args, **kwargs):
        from warnings import warn
        warn('importing the unstable item "{}" from gtsam is deprecated. Please import it from gtsam_unstable.'.format(name))
        return item(*args, **kwargs)
    return wrapper

for name in dir(gtsam_unstable):
    if not name.startswith('__'):
        item = getattr(gtsam_unstable, name)
        if callable(item):
            item = deprecated_wrapper(item, name)
        globals()[name] = item

