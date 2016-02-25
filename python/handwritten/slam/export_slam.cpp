/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   export_slam
 * @brief  wraps slam classes
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 * @author Frank Dellaert
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/slam/RotateFactor.h>

using namespace boost::python;
using namespace gtsam;
using namespace std;

void export_slam() {
  class_<RotateDirectionsFactor, bases<NonlinearFactor>>(
      "RotateDirectionsFactor",
      init<Key, const Unit3&, const Unit3&, noiseModel::Base::shared_ptr>())
      .def("Initialize", &RotateDirectionsFactor::Initialize)
      .staticmethod("Initialize");
}
