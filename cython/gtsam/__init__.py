from .gtsam import *

try:
    import gtsam_unstable


    def _deprecated_wrapper(item, name):
        def wrapper(*args, **kwargs):
            from warnings import warn
            message = ('importing the unstable item "{}" directly from gtsam is deprecated. '.format(name) +
                       'Please import it from gtsam_unstable.')
            warn(message)
            return item(*args, **kwargs)
        return wrapper


    for name in dir(gtsam_unstable):
        if not name.startswith('__'):
            item = getattr(gtsam_unstable, name)
            if callable(item):
                item = _deprecated_wrapper(item, name)
            globals()[name] = item

except ImportError:
    pass

