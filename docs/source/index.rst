.. GTSAM documentation master file, created by
   sphinx-quickstart on Tue Jul 21 14:09:31 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

GTSAM
======

GTSAM 4.0 is a BSD-licensed C++ library that implements sensor fusion for
robotics and computer vision applications, including
SLAM (Simultaneous Localization and Mapping), VO (Visual Odometry),
and SFM (Structure from Motion).
It uses factor graphs and Bayes networks as the underlying computing paradigm
rather than sparse matrices to optimize for the most probable configuration
or an optimal plan.
Coupled with a capable sensor front-end (not provided here), GTSAM powers
many impressive autonomous systems, in both academia and industry.


.. toctree::
   :caption: Getting started
   :hidden:

   Home<self>
   Installing
   Building


.. toctree::
   :caption: Contents
   :hidden:

   Tutorials
   Bindings
   C++ API <_static/doxygen/html/modules.html#http://>

.. (JLBC note: Do not remove the #http:// above, it's the only way I found to allow that link to be included in the TOC).


Indices and tables
==================

* :ref:`genindex`
