#!/usr/bin/env python

#http://docs.python.org/2/distutils/setupscript.html#setup-script

from distutils.core import setup

setup(name='GTSAM',
      version='3.0',
      description='Python Distribution Utilities',
      author='Dellaert et. al',
      author_email='Andrew.Melim@gatech.edu',
      url='http://www.python.org/sigs/distutils-sig/',
      packages=['gtsam'],
      package_data={'gtsam' : ['libgtsam.so']},
     )
