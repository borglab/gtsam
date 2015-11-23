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

// Wrapper on PinholeBaseK<Cal3_S2> because it has pure virtual method calibration()
struct PinholeBaseKCal3_S2Callback : PinholeBaseKCal3_S2, wrapper<PinholeBaseKCal3_S2>
{
  const Cal3_S2 & calibration () const {
    return this->get_override("calibration")();
  }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(project_overloads, PinholeBaseKCal3_S2::project, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(range_overloads, PinholeBaseKCal3_S2::range, 1, 3)

// Function pointers to desambiguate project() calls
Point2  (PinholeBaseKCal3_S2::*project1) (const Point3 &pw) const = &PinholeBaseKCal3_S2::project;
Point2  (PinholeBaseKCal3_S2::*project2) (const Point3 &pw, OptionalJacobian< 2, 6 > Dpose, OptionalJacobian< 2, 3 > Dpoint, OptionalJacobian< 2, FixedDimension<Cal3_S2>::value > Dcal) const = &PinholeBaseKCal3_S2::project;
Point2  (PinholeBaseKCal3_S2::*project3) (const Unit3 &pw,  OptionalJacobian< 2, 6 > Dpose, OptionalJacobian< 2, 2 > Dpoint, OptionalJacobian< 2, FixedDimension<Cal3_S2>::value > Dcal) const = &PinholeBaseKCal3_S2::project;

// function pointers to desambiguate range() calls
double (PinholeBaseKCal3_S2::*range1) (const Point3 &point, OptionalJacobian< 1, 6 > Dcamera, OptionalJacobian< 1, 3 > Dpoint) const = &PinholeBaseKCal3_S2::range;
double (PinholeBaseKCal3_S2::*range2) (const Pose3 &pose,   OptionalJacobian< 1, 6 > Dcamera, OptionalJacobian< 1, 6 > Dpose)  const = &PinholeBaseKCal3_S2::range;
double (PinholeBaseKCal3_S2::*range3) (const CalibratedCamera &camera, OptionalJacobian< 1, 6 > Dcamera, OptionalJacobian< 1, 6 > Dother) const = &PinholeBaseKCal3_S2::range;

void exportPinholeBaseK(){

  class_<PinholeBaseKCal3_S2Callback, boost::noncopyable>("PinholeBaseKCal3_S2", no_init)
    .def("calibration", pure_virtual(&PinholeBaseKCal3_S2::calibration), return_value_policy<copy_const_reference>())
    .def("project", project1)
    .def("project", project2, project_overloads())
    .def("project", project3, project_overloads())
    .def("backproject", &PinholeBaseKCal3_S2::backproject)
    .def("backproject_point_at_infinity", &PinholeBaseKCal3_S2::backprojectPointAtInfinity)
    .def("range", range1, range_overloads())
    .def("range", range2, range_overloads())
    .def("range", range3, range_overloads())
  ;

}