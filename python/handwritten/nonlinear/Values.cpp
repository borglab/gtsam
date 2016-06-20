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
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Values::print, 0, 1);

void exportValues(){

  typedef imuBias::ConstantBias Bias;

  bool (Values::*exists1)(Key) const = &Values::exists;
  void  (Values::*insert_point2)(Key, const gtsam::Point2&) = &Values::insert;
  void  (Values::*insert_rot2)  (Key, const gtsam::Rot2&) = &Values::insert;
  void  (Values::*insert_pose2) (Key, const gtsam::Pose2&) = &Values::insert;
  void  (Values::*insert_point3)(Key, const gtsam::Point3&) = &Values::insert;
  void  (Values::*insert_rot3)  (Key, const gtsam::Rot3&) = &Values::insert;
  void  (Values::*insert_pose3) (Key, const gtsam::Pose3&) = &Values::insert;
  void  (Values::*insert_bias) (Key, const Bias&) = &Values::insert;
  void  (Values::*insert_navstate) (Key, const NavState&) = &Values::insert;
  void  (Values::*insert_vector) (Key, const gtsam::Vector&) = &Values::insert;
  void  (Values::*insert_vector2) (Key, const gtsam::Vector2&) = &Values::insert;
  void  (Values::*insert_vector3) (Key, const gtsam::Vector3&) = &Values::insert;

  class_<Values>("Values", init<>())
  .def(init<Values>())
  .def("clear", &Values::clear)
  .def("dim", &Values::dim)
  .def("empty", &Values::empty)
  .def("equals", &Values::equals)
  .def("erase", &Values::erase)
  .def("print", &Values::print, print_overloads(args("s")))
  .def("size", &Values::size)
  .def("swap", &Values::swap)
  .def("insert", insert_point2)
  .def("insert", insert_rot2)
  .def("insert", insert_pose2)
  .def("insert", insert_point3)
  .def("insert", insert_rot3)
  .def("insert", insert_pose3)
  .def("insert", insert_bias)
  .def("insert", insert_navstate)
  .def("insert", insert_vector)
  .def("insert", insert_vector2)
  .def("insert", insert_vector3)
  .def("atPoint2", &Values::at<Point2>)
  .def("atRot2", &Values::at<Rot2>)
  .def("atPose2", &Values::at<Pose2>)
  .def("atPoint3", &Values::at<Point3>)
  .def("atRot3", &Values::at<Rot3>)
  .def("atPose3", &Values::at<Pose3>)
  .def("atConstantBias", &Values::at<Bias>)
  .def("atNavState", &Values::at<NavState>)
  .def("atVector", &Values::at<Vector>)
  .def("atVector2", &Values::at<Vector2>)
  .def("atVector3", &Values::at<Vector3>)
  .def("exists", exists1)
  .def("keys", &Values::keys)
  ;
}
