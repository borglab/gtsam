"""Setup file for the GTwrap package"""

try:
    from setuptools import find_packages, setup
except ImportError:
    from distutils.core import find_packages, setup

packages = find_packages()

setup(
    name='gtwrap',
    description='Library to wrap C++ with Python and Matlab',
    version='2.0.0',
    author="Frank Dellaert et. al.",
    author_email="dellaert@gatech.edu",
    license='BSD',
    keywords="wrap, bindings, cpp, python",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    python_requires=">=3.5",
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
    install_requires=open("requirements.txt").readlines(),
)
