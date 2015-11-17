/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file nonlinear_python.cpp
 * @brief wraps nonlinear classes into the nonlinear submodule of gtsam python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

 /** TODOs Summary:
  *
  */

#include <boost/python.hpp>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>

using namespace boost::python;
using namespace gtsam;

// Prototypes used to perform overloading
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html
void  (Values::*insert_0)(const gtsam::Values&) = &Values::insert;
void  (Values::*insert_1)(Key, const gtsam::Pose3&) = &Values::insert;

BOOST_PYTHON_MODULE(libnonlinear_python)
{

class_<Values>("Values")
  .def(init<>())
  .def(init<const Values&>())
  .def("clear", &Values::clear)
  .def("dim", &Values::dim)
  .def("empty", &Values::empty)
  .def("equals", &Values::equals)
  .def("erase", &Values::erase)
  .def("insertFixed", &Values::insertFixed)
  .def("print", &Values::print)
  .def("size", &Values::size)
  .def("swap", &Values::swap)
  .def("insert", insert_0)
  .def("insert", insert_1)
;

}