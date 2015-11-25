#!/usr/bin/env python

#http://docs.python.org/2/distutils/setupscript.html#setup-script

from distutils.core import setup

setup(name='gtsam',
      version='4.0.0',
      description='GTSAM Python wrapper',
      license = "BSD",
      author='Dellaert et. al',
      author_email='Andrew.Melim@gatech.edu',
      maintainer_email='gtsam@lists.gatech.edu',
      url='https://collab.cc.gatech.edu/borg/gtsam',
      packages=['gtsam', 'gtsam.examples', 'gtsam.utils'],
      package_data={'gtsam' : ['_libgtsam_python.so']},
     )
