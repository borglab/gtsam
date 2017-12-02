/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Cal3_S2 class to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/geometry/Cal3_S2.h>

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Cal3_S2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Cal3_S2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(uncalibrate_overloads, Cal3_S2::uncalibrate, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(calibrate_overloads, Cal3_S2::calibrate, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(between_overloads, Cal3_S2::between, 1, 3)

// Function pointers to desambiguate Cal3_S2::calibrate calls
Point2 (Cal3_S2::*calibrate1)(const Point2 &, OptionalJacobian< 2, 5 > Dcal, OptionalJacobian< 2, 2 > Dp) const = &Cal3_S2::calibrate;
Vector3 (Cal3_S2::*calibrate2)(const Vector3 &) const = &Cal3_S2::calibrate;

void exportCal3_S2(){

class_<Cal3_S2, boost::shared_ptr<Cal3_S2> >("Cal3_S2", init<>())
  .def(init<double,double,double,double,double>())
  .def(init<const Vector &>())
  .def(init<double,int,int>(args("fov","w","h")))
  .def(init<std::string>())
  .def(repr(self))
  .def("print", &Cal3_S2::print, print_overloads(args("s")))
  .def("equals", &Cal3_S2::equals, equals_overloads(args("q","tol")))
  .def("fx",&Cal3_S2::fx)
  .def("fy",&Cal3_S2::fy)
  .def("skew",&Cal3_S2::skew)
  .def("px",&Cal3_S2::px)
  .def("py",&Cal3_S2::py)
  .def("principal_point",&Cal3_S2::principalPoint)
  .def("vector",&Cal3_S2::vector)
  .def("k",&Cal3_S2::K)
  .def("matrix",&Cal3_S2::matrix)
  .def("matrix_inverse",&Cal3_S2::matrix_inverse)
  .def("uncalibrate",&Cal3_S2::uncalibrate, uncalibrate_overloads())
  .def("calibrate",calibrate1, calibrate_overloads())
  .def("calibrate",calibrate2)
  .def("between",&Cal3_S2::between, between_overloads())
;
register_ptr_to_python< boost::shared_ptr<Cal3_S2> >();

}
