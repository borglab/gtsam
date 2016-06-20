/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps PinholeCamera classes to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Cal3_S2.h"

using namespace boost::python;
using namespace gtsam;

typedef PinholeBaseK<Cal3_S2> PinholeBaseKCal3_S2;
typedef PinholeCamera<Cal3_S2> PinholeCameraCal3_S2;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, PinholeCameraCal3_S2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, PinholeCameraCal3_S2::equals, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Lookat_overloads, PinholeCameraCal3_S2::Lookat, 3, 4)

void exportPinholeCamera(){

class_<PinholeCameraCal3_S2, bases<PinholeBaseKCal3_S2> >("PinholeCameraCal3_S2", init<>())
  .def(init<const Pose3 &>())
  .def(init<const Pose3 &, const Cal3_S2 &>())
  .def(init<const Vector &>())
  .def(init<const Vector &, const Vector &>())
  .def("print", &PinholeCameraCal3_S2::print, print_overloads(args("s")))
  .def("equals", &PinholeCameraCal3_S2::equals, equals_overloads(args("q","tol")))
  .def("pose", &PinholeCameraCal3_S2::pose, return_value_policy<copy_const_reference>())
  // We don't need to define calibration() here because it's already defined as virtual in the base class PinholeBaseKCal3_S2
  // .def("calibration", &PinholeCameraCal3_S2::calibration, return_value_policy<copy_const_reference>())
  .def("Lookat", &PinholeCameraCal3_S2::Lookat, Lookat_overloads())
  .staticmethod("Lookat")
;

}