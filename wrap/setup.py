"""Setup file for the GTwrap package"""

import os
import sys

try:
    from setuptools import find_packages, setup
except ImportError:
    from distutils.core import find_packages, setup

packages = find_packages()

setup(
    name='gtwrap',
    description='Library to wrap C++ with Python and Matlab',
    version='1.0.0',
    author="",
    author_email="",
    license='',
    keywords="",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    python_requires=">=3.6",
    # https://pypi.org/classifiers
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Education',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Operating System :: MacOS',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development :: Libraries'
    ],
    packages=packages,
    platforms="any",
    # install_requires=["gtsam>=4", "opencv"],
)
