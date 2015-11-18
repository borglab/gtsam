/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file slam_python.cpp
 * @brief wraps slam classes into the slam submodule of gtsam python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

 /** TODOs Summary:
  *
  */

#include <boost/python.hpp>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

using namespace boost::python;
using namespace gtsam;

// Prototypes used to perform overloading
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html

// Macro used to define a BetweenFactor given the type.
#define BETWEENFACTOR(VALUE) \
  class_< BetweenFactor<VALUE> >("BetweenFactor"#VALUE) \
  .def(init<Key,Key,VALUE,noiseModel::Base::shared_ptr>()) \
  .def("measured", &BetweenFactor<VALUE>::measured, return_internal_reference<>()) \
;

BOOST_PYTHON_MODULE(libslam_python)
{

  BETWEENFACTOR(Point3)

  BETWEENFACTOR(Rot3)

  BETWEENFACTOR(Pose3)

}