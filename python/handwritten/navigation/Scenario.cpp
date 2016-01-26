/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps ConstantTwistScenario class to python
 * @author Frank Dellaert
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/navigation/Scenario.h"

using namespace boost::python;
using namespace gtsam;

void exportScenario() {
  // TODO(frank): figure out how to do inheritance
  class_<ConstantTwistScenario>("ConstantTwistScenario",
                                init<const Vector3&, const Vector3&>())
      .def("pose", &Scenario::pose)
      .def("omega_b", &Scenario::omega_b)
      .def("velocity_n", &Scenario::velocity_n)
      .def("acceleration_n", &Scenario::acceleration_n)
      .def("rotation", &Scenario::rotation)
      .def("velocity_b", &Scenario::velocity_b)
      .def("acceleration_b", &Scenario::acceleration_b);
}
