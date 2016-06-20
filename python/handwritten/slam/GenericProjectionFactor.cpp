/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps GenericProjectionFactor for several values to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/slam/ProjectionFactor.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Cal3_S2.h"

using namespace boost::python;
using namespace gtsam;

using namespace std;

typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> GenericProjectionFactorCal3_S2;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, GenericProjectionFactorCal3_S2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, GenericProjectionFactorCal3_S2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(evaluateError_overloads, GenericProjectionFactorCal3_S2::evaluateError, 2, 4)

void exportGenericProjectionFactor()
{

  class_<GenericProjectionFactorCal3_S2, bases<NonlinearFactor> >("GenericProjectionFactorCal3_S2", init<>())
    .def(init<const Point2 &, SharedNoiseModel, Key, Key, const boost::shared_ptr<Cal3_S2> &, optional<Pose3> >())
    .def(init<const Point2 &, SharedNoiseModel, Key, Key, const boost::shared_ptr<Cal3_S2> &, bool, bool, optional<Pose3> >())
    .def("print", &GenericProjectionFactorCal3_S2::print, print_overloads(args("s")))
    .def("equals", &GenericProjectionFactorCal3_S2::equals, equals_overloads(args("q","tol")))
    .def("evaluate_error", &GenericProjectionFactorCal3_S2::evaluateError, evaluateError_overloads())
    .def("measured", &GenericProjectionFactorCal3_S2::measured, return_value_policy<copy_const_reference>())
    // TODO(Ellon): Find the right return policy when returning a 'const shared_ptr<...> &'
    // .def("calibration", &GenericProjectionFactorCal3_S2::calibration, return_value_policy<copy_const_reference>())
    .def("verbose_cheirality", &GenericProjectionFactorCal3_S2::verboseCheirality)
    .def("throw_cheirality", &GenericProjectionFactorCal3_S2::throwCheirality)
  ;

}
