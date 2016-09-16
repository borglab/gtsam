from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import eigency



setup(
  ext_modules = cythonize(Extension(
    "gtsam",
    sources=["gtsam.pyx"],
    include_dirs = ["/Users/dta-huynh/install/include",
                    "/Users/dta-huynh/install/include/gtsam/3rdparty/Eigen",
                    "/usr/local/include"] + eigency.get_includes(include_eigen=False),

    libraries = ['gtsam'],
    library_dirs = ["/Users/dta-huynh/install/lib"],
    language="c++",
    extra_compile_args=["-std=c++11"])),
)