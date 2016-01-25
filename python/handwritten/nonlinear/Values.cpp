/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Values class to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/nonlinear/Values.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose3.h"

using namespace boost::python;
using namespace gtsam;

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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Values::print, 0, 1);

void exportValues(){

  // NOTE: Apparently the class 'Value'' is deprecated, so the commented lines below 
  //       will compile, but are useless in the python wrapper. We need to use specific 
  //       'at' and 'insert' methods for each type.
  // const Value& (Values::*at1)(Key) const = &Values::at;
  // void (Values::*insert1)(Key, const Value&) = &Values::insert;
  bool (Values::*exists1)(Key) const = &Values::exists;
  void  (Values::*insert_point2)(Key, const gtsam::Point2&) = &Values::insert;
  void  (Values::*insert_rot2)  (Key, const gtsam::Rot2&) = &Values::insert;
  void  (Values::*insert_pose2) (Key, const gtsam::Pose2&) = &Values::insert;
  void  (Values::*insert_point3)(Key, const gtsam::Point3&) = &Values::insert;
  void  (Values::*insert_rot3)  (Key, const gtsam::Rot3&) = &Values::insert;
  void  (Values::*insert_pose3) (Key, const gtsam::Pose3&) = &Values::insert;


  class_<Values>("Values", init<>())
  .def(init<Values>())
  .def("clear", &Values::clear)
  .def("dim", &Values::dim)
  .def("empty", &Values::empty)
  .def("equals", &Values::equals)
  .def("erase", &Values::erase)
  .def("insert_fixed", &Values::insertFixed)
  .def("print", &Values::print, print_overloads(args("s")))
  .def("size", &Values::size)
  .def("swap", &Values::swap)
  // NOTE: Following commented lines add useless methods on Values
  // .def("insert", insert1)
  // .def("at", at1, return_value_policy<copy_const_reference>())
  .def("insert", insert_point2)
  .def("insert", insert_rot2)
  .def("insert", insert_pose2)
  .def("insert", insert_point3)
  .def("insert", insert_rot3)
  .def("insert", insert_pose3)
  // NOTE: The following commented lines are another way of specializing the return type.
  //       See long comment above.
  // .def("at", &ValuesAt<Point3>, return_internal_reference<>())
  // .def("at", &ValuesAt<Rot3>, return_internal_reference<>())
  // .def("at", &ValuesAt<Pose3>, return_internal_reference<>())
  .def("point3_at", &Values::at<Point3>, return_value_policy<copy_const_reference>())
  .def("rot3_at", &Values::at<Rot3>, return_value_policy<copy_const_reference>())
  .def("pose3_at", &Values::at<Pose3>, return_value_policy<copy_const_reference>())  
  .def("exists", exists1)
  .def("keys", &Values::keys)
  ;
}
