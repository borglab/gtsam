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
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/geometry/Pose3.h>

using namespace boost::python;
using namespace gtsam;

// Overloading macros
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html
// ISAM2
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ISAM2_update_overloads, ISAM2::update, 0, 7)


/** The function ValuesAt is a workaround to be able to call the correct templated version
  * of Values::at. Without it, python would only try to match the last 'at' metho defined
  * below. With this wrapper function we can call 'at' in python passing an extra type,
  * which will define the type to be returned. Example:
  *
  *     >>> import gtsam
  *     >>> v = gtsam.nonlinear.Values()
  *     >>> v.insert(1,gtsam.geometry.Point3())
  *     >>> v.insert(2,gtsam.geometry.Rot3())
  *     >>> v.insert(3,gtsam.geometry.Pose3())
  *     >>> v.at(1,gtsam.geometry.Point3())
  *     >>> v.at(2,gtsam.geometry.Rot3())
  *     >>> v.at(3,gtsam.geometry.Pose3())
  *
  * A more 'pythonic' way I think would be to not use this function and define different 
  * 'at' methods below using the name of the type in the function name, like:
  *
  *     .def("point3_at", &Values::at<Point3>, return_internal_reference<>())
  *     .def("rot3_at", &Values::at<Rot3>, return_internal_reference<>())
  *     .def("pose3_at", &Values::at<Pose3>, return_internal_reference<>())
  *
  * and then they could be accessed from python as 
  * 
  *     >>> import gtsam
  *     >>> v = gtsam.nonlinear.Values()
  *     >>> v.insert(1,gtsam.geometry.Point3())
  *     >>> v.insert(2,gtsam.geometry.Rot3())
  *     >>> v.insert(3,gtsam.geometry.Pose3())
  *     >>> v.point3_at(1)
  *     >>> v.rot3_at(2)
  *     >>> v.pose3_at(3)
  *
  * In fact, I just saw the pythonic way sounds more clear, so I'm sticking with this and
  * leaving the comments here for future reference. I'm using the PEP0008 for method naming.
  * See: https://www.python.org/dev/peps/pep-0008/#function-and-method-arguments
  */
// template<typename T>
// const T  & ValuesAt( const Values & v, Key j, T /*type*/)
// {
//   return v.at<T>(j);
// }

BOOST_PYTHON_MODULE(libnonlinear_python)
{

// Function pointers for overloads in Values
void  (Values::*insert_0)(const gtsam::Values&) = &Values::insert;
void  (Values::*insert_1)(Key, const gtsam::Point3&) = &Values::insert;
void  (Values::*insert_2)(Key, const gtsam::Rot3&) = &Values::insert;
void  (Values::*insert_3)(Key, const gtsam::Pose3&) = &Values::insert;

class_<Values>("Values")
  .def(init<>())
  .def(init<const Values&>())
  .def("clear", &Values::clear)
  .def("dim", &Values::dim)
  .def("empty", &Values::empty)
  .def("equals", &Values::equals)
  .def("erase", &Values::erase)
  .def("insert_fixed", &Values::insertFixed)
  .def("print", &Values::print)
  .def("size", &Values::size)
  .def("swap", &Values::swap)
  .def("insert", insert_0)
  .def("insert", insert_1)
  .def("insert", insert_2)
  .def("insert", insert_3)
  // .def("at", &ValuesAt<Point3>, return_internal_reference<>())
  // .def("at", &ValuesAt<Rot3>, return_internal_reference<>())
  // .def("at", &ValuesAt<Pose3>, return_internal_reference<>())
  .def("point3_at", &Values::at<Point3>, return_internal_reference<>())
  .def("rot3_at", &Values::at<Rot3>, return_internal_reference<>())
  .def("pose3_at", &Values::at<Pose3>, return_internal_reference<>())
;

// Function pointers for overloads in NonlinearFactorGraph
void (NonlinearFactorGraph::*add_0)(const NonlinearFactorGraph::sharedFactor&) = &NonlinearFactorGraph::add;

class_<NonlinearFactorGraph>("NonlinearFactorGraph")
  .def("size",&NonlinearFactorGraph::size)
  .def("add", add_0)
;

// TODO(Ellon): Export all properties of ISAM2Params
class_<ISAM2Params>("ISAM2Params")
;

// TODO(Ellon): Export useful methods/properties of ISAM2Result
class_<ISAM2Result>("ISAM2Result")
;

// Function pointers for overloads in ISAM2
Values (ISAM2::*calculateEstimate_0)() const = &ISAM2::calculateEstimate;

class_<ISAM2>("ISAM2")
  // TODO(Ellon): wrap all optional values of update
  .def("update",&ISAM2::update, ISAM2_update_overloads())
  .def("calculate_estimate", calculateEstimate_0)
;

}