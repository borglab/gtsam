/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   export_geometry
 * @brief  wraps geometry classes
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 * @author Frank Dellaert
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/geometry/Unit3.h>

using namespace boost::python;
using namespace gtsam;
using namespace std;

void export_geometry() {
  class_<Unit3>("Unit3")
      .def(init<>())
      .def(init<double, double, double>())
      .def(init<const Vector3&>());
}
