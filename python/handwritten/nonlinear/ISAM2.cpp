/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief exports ISAM2 class to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/nonlinear/ISAM2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(update_overloads, ISAM2::update, 0, 7)

void exportISAM2(){

// TODO(Ellon): Export all properties of ISAM2Params
class_<ISAM2Params>("ISAM2Params")
;

// TODO(Ellon): Export useful methods/properties of ISAM2Result
class_<ISAM2Result>("ISAM2Result")
;

// Function pointers for overloads in ISAM2
Values (ISAM2::*calculateEstimate_0)() const = &ISAM2::calculateEstimate;

class_<ISAM2>("ISAM2")
  .def(init<const ISAM2Params &>())
  // TODO(Ellon): wrap all optional values of update
  .def("update",&ISAM2::update, update_overloads())
  .def("calculate_estimate", calculateEstimate_0)
;

}